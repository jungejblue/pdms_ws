# 끝점 처리와 뷰어 간격

GT 경로 모드에서 GT의 투영이 정상이고 모델만 기준 경로 끝점을 초과하면
평가를 계속합니다. 기존 최종 진행량 비율을 [0, 1]로 제한합니다.
중간에 초과했다가 되돌아온 경우 최종 진행량으로 계산하며 EP=1을 고정하지 않습니다.
GT 끝점 초과는 `EP GT baseline uncertain`으로 invalid 기록합니다.
분기 투영 모호성, 정지 GT 경로 투영 제한, centerline 모드 끝점 검사는 유지합니다.
저진행 EP 규칙, TTC 등 다른 점수 계산식은 변경하지 않습니다.

result.json에는 model_endpoint_exceeded, gt_endpoint_exceeded가 추가됩니다.
GT/모델 점수 계산이 완료된 경우 invalid여도 diagnostics.json을 보존합니다.
새 결과에는 sample_start_time_s (raw 기준 초 단위 시작 시각)를 저장합니다.

```bash
pdms evaluate --out nuscenes_endpoint_check --limit 10 --workers 1
pdms serve --run nuscenes_endpoint_check --period 1.5
pdms evaluate --out nuscenes_endpoint_all --workers 4
pdms serve --run nuscenes_endpoint_all --period 1.0
pdms serve --run nuscenes_endpoint_all --period 1.5
pdms serve --run nuscenes_endpoint_all
```

period 기본 0: 기존 전체 목록/점수순 표시를 유지합니다.
period 양수: valid이고 scene.json/trajectories.npz가 있는 sample을 scene별 시작 시각순으로
정렬하고 첫 sample부터 이전 선택과 최소 period 초 차이 나는 첫 sample을 선택합니다.
선택 목록은 scene/시간순으로 표시됩니다. 원래 sample이 없으면 다음 가능한 sample을
사용하며 새 frame을 보간 생성하지 않습니다. scene마다 간격 기준을 초기화합니다.
각 sample 내부의 3초/0.1초 재생과 평가 점수/집계/파일은 바뀌지 않습니다.
이전/다음 및 token 검색은 선택된 목록 안에서 동작합니다.
음수, NaN, 무한대 period는 오류입니다.

기존 결과에 sample_start_time_s가 없으면 period 양수는 안내 후 중단합니다.
기존 결과는 period 0으로 열거나 업데이트 후 새 out에 재평가하세요.
시각화 간격 변경에는 새 결과를 한 번 만든 뒤 추가 평가가 필요하지 않습니다.
기본 평가 대상은 모든 시간 조건 충족 sample이며 worker 최대 4, 포트 7200입니다.
