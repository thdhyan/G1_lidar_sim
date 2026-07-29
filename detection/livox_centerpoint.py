"""livox_detection ported to modern PyTorch.

The upstream project (Livox-SDK/livox_detection) targets Python 3.8 / torch
1.8.2 / CUDA 10.2, which cannot be installed on an Ada-generation GPU. Only the
checkpoints are reused here; the network is re-declared against current torch.

Architecture was recovered from ``pt/livox_model_1.pt`` rather than guessed:

* input  - 30-channel bird's-eye-view binary occupancy map
* stem   - 5 stride-2 blocks, channels 30 -> 32 -> 48 -> 64 -> 96 -> 128, each
           followed by [2, 2, 3, 3, 2] bottleneck residual units
* neck   - each block output upsampled back to input resolution by its stride
           [2, 4, 8, 16, 32] and concatenated (32+48+64+96+128 = 368), fused by
           a 1x1 conv to 128 channels, then re-weighted by a channel softmax
* head   - CenterPoint: shared 3x3 conv to 64, then hm(3) / center(2) /
           center_z(1) / dim(3) / rot(2)

Layer definitions are transcribed from upstream ``resfpn.py`` / ``boolmap.py``
so the published checkpoints load with zero missing or unexpected keys.

Everything is plain Conv2d/BatchNorm2d - there are no spconv or custom CUDA
ops, so this runs anywhere torch runs.
"""

from __future__ import annotations

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

from backend import CLASS_NAMES, DetectionBackend, _empty

# Detection volume in the LiDAR frame, from the upstream config.
POINT_CLOUD_RANGE = (0.0, -44.8, -2.0, 224.0, 44.8, 4.0)
VOXEL_SIZE = (0.2, 0.2, 0.2)
# 30 input channels = the 30 z-slices of the occupancy volume.
NUM_HEIGHT_BINS = 30


class Bottleneck(nn.Module):
    """1x1 -> 3x3 -> 1x1 residual unit with identity shortcut.

    Indices of the inner ``Sequential`` are load-bearing: they must line up with
    the ``residual_function.{0,3,6}`` conv and ``{1,4,7}`` norm keys in the
    checkpoint, hence the explicit ReLU entries at positions 2 and 5.

    Upstream only adds a projection shortcut when the shape changes; every unit
    here is channel-preserving and stride-1, so the shortcut is always identity
    and carries no parameters.
    """

    def __init__(self, channels: int):
        super().__init__()
        self.residual_function = nn.Sequential(
            nn.Conv2d(channels, channels, 1, bias=False),
            nn.BatchNorm2d(channels),
            nn.ReLU(inplace=True),
            nn.Conv2d(channels, channels, 3, padding=1, bias=False),
            nn.BatchNorm2d(channels),
            nn.ReLU(inplace=True),
            nn.Conv2d(channels, channels, 1, bias=False),
            nn.BatchNorm2d(channels),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return F.relu(x + self.residual_function(x), inplace=True)


def _make_block(in_ch: int, out_ch: int, num_units: int, stride: int) -> nn.Sequential:
    """One backbone stage: pad, strided conv, norm, relu, then residual units.

    Index 0 is the padding layer (no parameters) so that the conv lands on index
    1 and the norm on index 2, matching the checkpoint's ``blocks.N.1`` /
    ``blocks.N.2`` keys. The BatchNorm eps/momentum match upstream; the defaults
    would shift the normalisation of the pretrained running statistics.
    """
    layers: list[nn.Module] = [
        nn.ZeroPad2d(1),
        nn.Conv2d(in_ch, out_ch, 3, stride=stride, padding=0, bias=False),
        nn.BatchNorm2d(out_ch, eps=1e-3, momentum=0.01),
        nn.ReLU(),
    ]
    layers += [Bottleneck(out_ch) for _ in range(num_units)]
    return nn.Sequential(*layers)


class Backbone(nn.Module):
    """Multi-scale BEV feature extractor with attention-gated fusion."""

    CHANNELS = (32, 48, 64, 96, 128)
    UNITS = (2, 2, 3, 3, 2)
    STRIDES = (2, 2, 2, 2, 2)
    # Each level is upsampled by its cumulative stride, so all five land back at
    # the input resolution before being concatenated.
    UPSAMPLE_STRIDES = (2, 4, 8, 16, 32)

    def __init__(self, in_channels: int = NUM_HEIGHT_BINS):
        super().__init__()
        blocks = []
        prev = in_channels
        for ch, units, stride in zip(self.CHANNELS, self.UNITS, self.STRIDES):
            blocks.append(_make_block(prev, ch, units, stride))
            prev = ch
        self.blocks = nn.ModuleList(blocks)
        self.deblocks = nn.ModuleList(
            [nn.Sequential(nn.UpsamplingBilinear2d(scale_factor=s)) for s in self.UPSAMPLE_STRIDES]
        )

        fused_in = sum(self.CHANNELS)  # 368
        self.fushion = nn.Sequential(  # spelling matches the checkpoint keys
            nn.Conv2d(fused_in, 128, 1, bias=False),
            nn.BatchNorm2d(128),
            nn.ReLU(inplace=True),
        )
        self.attention_w = nn.Sequential(
            nn.Conv2d(128, 128, 1, bias=False),
            nn.BatchNorm2d(128),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        ups = []
        for block, deblock in zip(self.blocks, self.deblocks):
            x = block(x)
            ups.append(deblock(x))

        fused = self.fushion(torch.cat(ups, dim=1))
        # Softmax across channels, not a sigmoid gate: it redistributes weight
        # between feature channels rather than scaling each independently.
        return torch.softmax(self.attention_w(fused), dim=1) * fused


class CenterHead(nn.Module):
    """CenterPoint head: a heatmap plus box-regression branches."""

    def __init__(self, in_channels: int = 128, num_classes: int = len(CLASS_NAMES)):
        super().__init__()
        self.shared_conv = nn.Sequential(
            nn.Conv2d(in_channels, 64, 3, padding=1),
            nn.BatchNorm2d(64),
            nn.ReLU(inplace=True),
        )
        # Wrapped in Sequential so keys read ``<name>.0.weight`` as in the checkpoint.
        make = lambda out: nn.Sequential(nn.Conv2d(64, out, 3, padding=1))
        self.heads_list = nn.ModuleList(
            [
                nn.ModuleDict(
                    {
                        "hm": make(num_classes),
                        "center": make(2),
                        "center_z": make(1),
                        "dim": make(3),
                        "rot": make(2),
                    }
                )
            ]
        )

    def forward(self, x: torch.Tensor) -> dict[str, torch.Tensor]:
        shared = self.shared_conv(x)
        head = self.heads_list[0]
        return {name: head[name](shared) for name in ("hm", "center", "center_z", "dim", "rot")}


class LivoxNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.backbone = Backbone()
        self.head = CenterHead()

    def forward(self, x: torch.Tensor) -> dict[str, torch.Tensor]:
        return self.head(self.backbone(x))


def points_to_bev(points: np.ndarray, device: torch.device) -> torch.Tensor:
    """Voxelise ``(N, 4)`` points into a ``(1, 30, H, W)`` occupancy volume."""
    x_min, y_min, z_min, x_max, y_max, z_max = POINT_CLOUD_RANGE
    vx, vy, vz = VOXEL_SIZE

    width = int(round((x_max - x_min) / vx))
    height = int(round((y_max - y_min) / vy))

    xyz = points[:, :3]
    inside = (
        (xyz[:, 0] >= x_min)
        & (xyz[:, 0] < x_max)
        & (xyz[:, 1] >= y_min)
        & (xyz[:, 1] < y_max)
        & (xyz[:, 2] >= z_min)
        & (xyz[:, 2] < z_max)
    )
    xyz = xyz[inside]

    bev = torch.zeros((1, NUM_HEIGHT_BINS, height, width), dtype=torch.float32, device=device)
    if xyz.shape[0] == 0:
        return bev

    xi = ((xyz[:, 0] - x_min) / vx).astype(np.int64)
    yi = ((xyz[:, 1] - y_min) / vy).astype(np.int64)
    zi = np.clip(((xyz[:, 2] - z_min) / vz).astype(np.int64), 0, NUM_HEIGHT_BINS - 1)

    idx = torch.from_numpy(np.stack([zi, yi, xi])).to(device)
    bev[0, idx[0], idx[1], idx[2]] = 1.0
    return bev


def _nms_bev(boxes: np.ndarray, scores: np.ndarray, threshold: float = 0.2) -> list[int]:
    """Greedy NMS on axis-aligned BEV footprints.

    Ignoring yaw makes the overlap slightly conservative, which is acceptable
    for suppressing duplicate peaks off the heatmap.
    """
    if boxes.shape[0] == 0:
        return []
    x1, y1 = boxes[:, 0] - boxes[:, 3] / 2, boxes[:, 1] - boxes[:, 4] / 2
    x2, y2 = boxes[:, 0] + boxes[:, 3] / 2, boxes[:, 1] + boxes[:, 4] / 2
    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort()[::-1]

    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(int(i))
        if order.size == 1:
            break
        rest = order[1:]
        w = np.maximum(0.0, np.minimum(x2[i], x2[rest]) - np.maximum(x1[i], x1[rest]))
        h = np.maximum(0.0, np.minimum(y2[i], y2[rest]) - np.maximum(y1[i], y1[rest]))
        overlap = w * h
        iou = overlap / (areas[i] + areas[rest] - overlap + 1e-6)
        order = rest[iou <= threshold]
    return keep


class LivoxCenterPointBackend(DetectionBackend):
    """Runs the ported livox_detection network."""

    def __init__(
        self,
        checkpoint: str,
        device: str = "cuda",
        score_threshold: float = 0.4,
        topk: int = 100,
        nms_threshold: float = 0.1,
    ):
        self.checkpoint = checkpoint
        self.device = torch.device(device if torch.cuda.is_available() else "cpu")
        self.score_threshold = score_threshold
        self.topk = topk
        self.nms_threshold = nms_threshold
        self.model: LivoxNet | None = None

    def load(self) -> None:
        state = torch.load(self.checkpoint, map_location="cpu", weights_only=False)
        state = state.get("model_state_dict", state)
        # Checkpoints were saved from a DataParallel wrapper.
        state = {k.replace("module.", "", 1): v for k, v in state.items()}

        model = LivoxNet()
        missing, unexpected = model.load_state_dict(state, strict=False)
        if missing or unexpected:
            # Surfaced rather than swallowed: a mismatch means the reconstructed
            # topology is wrong and any detections would be meaningless.
            raise RuntimeError(
                f"checkpoint does not match model definition "
                f"({len(missing)} missing, {len(unexpected)} unexpected). "
                f"first missing={missing[:3]} first unexpected={unexpected[:3]}"
            )

        self.model = model.to(self.device).eval()

    @torch.no_grad()
    def infer(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        if self.model is None:
            raise RuntimeError("load() must be called before infer()")
        if points.shape[0] == 0:
            return _empty()

        bev = points_to_bev(points, self.device)
        out = self.model(bev)

        heatmap = out["hm"][0].sigmoid()
        # The neck restores the input resolution, so feature cells map 1:1 onto
        # voxels and no extra stride factor is applied when decoding centres.
        num_classes, height, width = heatmap.shape

        # Local-maximum suppression on the heatmap, the usual CenterPoint
        # stand-in for a first NMS pass.
        pooled = F.max_pool2d(heatmap.unsqueeze(0), 3, stride=1, padding=1)[0]
        peaks = heatmap * (pooled == heatmap).float()

        flat = peaks.reshape(num_classes, -1)
        scores_flat, idx_flat = flat.reshape(-1).topk(min(self.topk, flat.numel()))
        keep_topk = scores_flat > self.score_threshold
        if not keep_topk.any():
            return _empty()
        scores_flat, idx_flat = scores_flat[keep_topk], idx_flat[keep_topk]

        cls_ids = idx_flat // (height * width)
        cell = idx_flat % (height * width)
        ys, xs = cell // width, cell % width

        centre = out["center"][0].reshape(2, -1)[:, cell]
        centre_z = out["center_z"][0].reshape(1, -1)[:, cell]
        dims = out["dim"][0].reshape(3, -1)[:, cell].exp()
        rot = out["rot"][0].reshape(2, -1)[:, cell]

        x_min, y_min = POINT_CLOUD_RANGE[0], POINT_CLOUD_RANGE[1]
        bx = (xs.float() + centre[0]) * VOXEL_SIZE[0] + x_min
        by = (ys.float() + centre[1]) * VOXEL_SIZE[1] + y_min
        # Channel 0 is cos and channel 1 is sin, matching upstream's decode.
        angle = torch.atan2(rot[1], rot[0])

        boxes = (
            torch.stack([bx, by, centre_z[0], dims[0], dims[1], dims[2], angle], dim=1)
            .cpu()
            .numpy()
            .astype(np.float32)
        )
        scores = scores_flat.cpu().numpy().astype(np.float32)
        labels = cls_ids.cpu().numpy().astype(np.int64)

        keep = _nms_bev(boxes, scores, threshold=self.nms_threshold)
        return boxes[keep], scores[keep], labels[keep]
