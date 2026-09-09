#!/usr/bin/env python3
"""
dino_seg.py — DINOv3(폴백 DINOv2) 패치 특징 추출 + 무라벨 표면 군집화

졸작 v2의 "특징점이 뭘 보는가" 설명용 코어 모듈.
  - DINO ViT는 16x16(또는 14x14) 패치마다 384차원 벡터를 낸다.
  - 이 벡터를 PCA로 3차원 → RGB 시각화하면 "도로 / 차 / 건물 / 잔디"가
    학습 라벨 없이도 색으로 분리됨 → 특징점이 표면의 의미를 담는다는 근거.
  - K-means로 군집화하면 각 패치를 표면 종류로 나눌 수 있다.

단독 실행: python3 dino_seg.py some_frame.png   →  some_frame_pca.png / some_frame_clusters.png
"""
import sys
import numpy as np
import torch
import torch.nn.functional as F
from PIL import Image

_IMAGENET_MEAN = torch.tensor([0.485, 0.456, 0.406]).view(1, 3, 1, 1)
_IMAGENET_STD = torch.tensor([0.229, 0.224, 0.225]).view(1, 3, 1, 1)


def _try_load(repo, name):
    m = torch.hub.load(repo, name, trust_repo=True)
    return m


class DinoSeg:
    def __init__(self, prefer="dinov2", device=None):  # dinov3 weights are license-gated (403)
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.model = None
        self.name = None
        attempts = []
        if prefer == "dinov3":
            attempts = [("facebookresearch/dinov3", "dinov3_vits16", 16),
                        ("facebookresearch/dinov2", "dinov2_vits14", 14)]
        else:
            attempts = [("facebookresearch/dinov2", "dinov2_vits14", 14),
                        ("facebookresearch/dinov3", "dinov3_vits16", 16)]
        last_err = None
        for repo, name, ps in attempts:
            try:
                self.model = _try_load(repo, name).to(self.device).eval()
                self.name = name
                self.patch = ps
                break
            except Exception as e:  # noqa
                last_err = e
                print(f"[dino_seg] {name} 로드 실패: {e}")
        if self.model is None:
            raise RuntimeError(f"DINO 로드 전부 실패: {last_err}")
        self.feat_dim = 384
        print(f"[dino_seg] 로드 완료: {self.name} (patch={self.patch}, dev={self.device})")

    @torch.no_grad()
    def extract(self, rgb_uint8, side=448):
        """rgb_uint8: HxWx3 uint8 → (patch_feats [Hp,Wp,C] float32, (Hp,Wp))"""
        side = (side // self.patch) * self.patch
        img = Image.fromarray(rgb_uint8).convert("RGB").resize((side, side), Image.BILINEAR)
        x = torch.from_numpy(np.asarray(img)).permute(2, 0, 1).float().div(255.0).unsqueeze(0)
        x = (x - _IMAGENET_MEAN) / _IMAGENET_STD
        x = x.to(self.device)
        out = self.model.forward_features(x)
        tok = out["x_norm_patchtokens"][0]            # [Np, C]
        hp = wp = side // self.patch
        feats = tok.reshape(hp, wp, -1).float().cpu().numpy()
        return feats, (hp, wp)

    @staticmethod
    def pca_rgb(feats):
        """feats [Hp,Wp,C] → [Hp,Wp,3] uint8 (상위 3 주성분을 0~255로 정규화)"""
        hp, wp, c = feats.shape
        x = torch.from_numpy(feats.reshape(-1, c))
        x = x - x.mean(0, keepdim=True)
        u, s, v = torch.pca_lowrank(x, q=3)
        proj = x @ v[:, :3]
        proj = (proj - proj.min(0).values) / (proj.max(0).values - proj.min(0).values + 1e-8)
        return (proj.reshape(hp, wp, 3).numpy() * 255).astype(np.uint8)

    @staticmethod
    def kmeans(feats, k=5, iters=25, seed=0):
        """feats [Hp,Wp,C] → labels [Hp,Wp] int, centers [k,C] (cosine 정규화 후 L2)"""
        hp, wp, c = feats.shape
        x = torch.from_numpy(feats.reshape(-1, c)).float()
        x = F.normalize(x, dim=1)
        g = torch.Generator().manual_seed(seed)
        idx = torch.randperm(x.shape[0], generator=g)[:k]
        cen = x[idx].clone()
        for _ in range(iters):
            d = torch.cdist(x, cen)              # [N,k]
            lab = d.argmin(1)
            for j in range(k):
                m = lab == j
                if m.any():
                    cen[j] = F.normalize(x[m].mean(0), dim=0)
        return lab.reshape(hp, wp).numpy(), cen.numpy()


def main():
    if len(sys.argv) < 2:
        print("사용법: python3 dino_seg.py <image.png>")
        return
    path = sys.argv[1]
    rgb = np.asarray(Image.open(path).convert("RGB"))
    seg = DinoSeg()
    feats, (hp, wp) = seg.extract(rgb)
    print(f"패치 격자: {hp}x{wp}, 차원 {feats.shape[-1]}")
    pca = seg.pca_rgb(feats)
    lab, _ = seg.kmeans(feats, k=5)
    # 시각화 저장 (원본 크기로 업샘플)
    H, W = rgb.shape[:2]
    pca_up = np.asarray(Image.fromarray(pca).resize((W, H), Image.NEAREST))
    palette = np.array([[230, 80, 60], [60, 180, 75], [70, 130, 230],
                        [240, 200, 40], [160, 90, 200]], np.uint8)
    clus_up = np.asarray(Image.fromarray(palette[lab]).resize((W, H), Image.NEAREST))
    base = path.rsplit(".", 1)[0]
    Image.fromarray(pca_up).save(base + "_pca.png")
    Image.fromarray(clus_up).save(base + "_clusters.png")
    print(f"저장: {base}_pca.png , {base}_clusters.png")


if __name__ == "__main__":
    main()
