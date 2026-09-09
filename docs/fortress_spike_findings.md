# Fortress(Gazebo Sim / ign-gazebo6) 타당성 스파이크 — 결론: 이 환경에선 불가

**날짜**: 2026-09-04
**배경**: Gazebo Sim(Harmonic 데모 스크린샷)처럼 사실적인 PBR 렌더링을 원해서, 이 WSL2 환경에
이미 설치돼 있던 Gazebo Fortress(ign-gazebo6, ROS 2 Humble 공식 짝) + ros_gz_bridge로 전체
포팅(2~4일 추정)을 하기 전에, 카메라 센서 하나만 띄워서 되는지부터 확인하는 스파이크.

## 결론

**카메라/depth 센서를 쓰는 순간(Ogre2/Ogre-Next 렌더러가 씬을 만들 때) 100% 크래시난다.**
`-s`(헤드리스 서버), `--headless-rendering` 유무 모두 동일하게 실패. 반대로 센서가 아예 없는
데모 월드(`shapes.sdf`)는 문제없이 잘 돈다 — 즉 물리 시뮬레이션 자체가 아니라 **카메라 렌더링
경로에서만** 터진다. 우리 프로젝트는 RGB+depth 카메라가 핵심이라 이 버그를 피해갈 방법이 없다.

```
terminate called after throwing an instance of 'Ogre::UnimplementedException'
  what():  OGRE EXCEPTION(9:UnimplementedException):  in GL3PlusTextureGpu::copyTo
    at .../RenderSystems/GL3Plus/src/OgreGL3PlusTextureGpu.cpp (line 677)
```

콜스택: `SensorsPrivate::RenderThread → RenderUtil::Init → CreateScene → CreateMaterials →
Ogre2Material::SetTextureMapImpl → TextureGpuManager::_waitFor/_update →
GenerateHwMipmaps::_executeSerial → RenderSystem_GL3Plus`

원인 추정: Ogre-Next(2.2.5)의 GL3Plus 렌더 시스템이 (WSLg의 Mesa/D3D12 변환 GL 스택 위에서)
카메라 렌더타겟의 밉맵 생성에 필요한 텍스처 copy 연산을 구현하지 않은 상태로 예외를 던짐.
Gazebo Classic 11이 쓰는 Ogre 1.9(GL 경로가 다름)에서는 이 문제가 없었던 것과 대조적 —
그래서 지금까지 Classic 데모는 전부 문제없이 돌아간 것.

## 회피 경로 확인 결과 — 전부 막힘

- Ogre-Next에 컴파일된 렌더 시스템: `RenderSystem_GL3Plus.so`, `RenderSystem_NULL.so` 뿐 —
  **Vulkan 백엔드가 아예 없음**(vulkaninfo도 미설치). GL3Plus를 못 쓰면 대안이 없다.
- `ign-rendering6`에 `ogre`(구 Ogre1 기반) 엔진 플러그인도 있긴 하지만, 그건 결국 Classic과
  같은 시각적 한계(PBR 없음)라 애초에 원했던 "사실적인 그림"이 안 나옴 — 시도할 실익이 없어서
  스킵.

## 결론 및 권고

- **지금 이 WSL 환경에서는 Fortress로 카메라 기반 프로젝트를 못 옮긴다.** 코드를 며칠 들여
  포팅해봤자 첫 카메라 센서에서 크래시라 애초에 의미가 없음 — 스파이크 덕분에 하루 이상
  걸렸을 전체 포팅을 10분 만에 걸렀다.
- 고칠 수 있는 종류의 문제가 아니다 — Ogre-Next 자체를 패치/재빌드하거나, WSL의 GPU
  드라이버/Mesa 스택이 바뀌거나, 더 최신 Gazebo(Harmonic/Ionic, gz-rendering8 — 이 버그가
  고쳐졌을 수도 있음, 근데 ROS 2 Humble 공식 짝은 아님, Iron/Jazzy용)로 가야 하는데 셋 다
  범위 밖.
- **권고: Gazebo Classic 11 유지.** 시각적 사실감 갭은 인정하지만, 대안이 이 환경에서 실제로
  동작하지 않는다는 게 실측으로 확인됐다. 발표 데모는 지금 검증된 파이프라인(격자도시/
  맨해튼/마을)으로 진행하고, "예쁜 그림"이 꼭 필요하면 Gazebo 바깥(Blender 등)에서 별도
  쇼케이스 영상을 만드는 쪽이 더 현실적이다.

## 재현 방법 (나중에 WSL/드라이버가 바뀌어서 재시도하고 싶을 때)

```bash
source /opt/ros/humble/setup.bash
ign gazebo -s -r /usr/share/ignition/ignition-gazebo6/worlds/depth_camera_sensor.sdf
# → 위와 동일한 Ogre::UnimplementedException으로 크래시하면 아직도 안 되는 것.
# 크래시 안 하면 그때 ros_gz_bridge로 실제 포팅 스파이크를 이어서 해볼 것.
```
