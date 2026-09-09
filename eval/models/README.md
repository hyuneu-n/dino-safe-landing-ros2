# eval/models/

`openlander_end2end.onnx` 는 여기 없음 — git에 안 커밋함 (11MB 바이너리, 재현 가능하면
저장소에 안 넣는다는 방침, CONTEXT.md 결정 2 "재현성=Docker+README" 참고).

받으려면:

```bash
bash download_openlander.sh
```

## 이 파일이 뭔지

- 원본 리포: https://github.com/stephansturges/OpenLander (MIT)
- 실제 가중치 출처: https://huggingface.co/spaces/StephanST/OpenLanderONNXonline
  (Streamlit 데모 앱의 리소스 파일)
- 왜 GitHub 리포에서 바로 안 받았나: 그 리포의 `models/*.blob` 은 **Luxonis OAK 카메라의
  Myriad X VPU 전용으로 컴파일된 바이너리**라서, onnxruntime은 물론 어떤 일반 CPU/GPU에서도
  못 돌린다. README가 "ONNX 모델"이라고 부르는 진짜 이식 가능한 `.onnx` 파일은 위 HuggingFace
  Space에만 올라와 있다 (app.py 참고).
- 사용 모델: 3개 중 "Embedded model better trained: DeeplabV3+, MobilenetV2, 416px" —
  app.py 라디오 버튼에서 저자가 "better trained"라고 명시한 것.
- 라이선스: MIT (레포 LICENSE 파일 그대로).
