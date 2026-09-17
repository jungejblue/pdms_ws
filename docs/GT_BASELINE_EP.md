# GT 고정 baseline EP

정규화: GT 경로 MPC rollout의 진행 거리 d_gt를 고정 분모로 사용합니다.
EP_gt=1; EP_model=clip(d_model/d_gt,0,1). 기준선(gt_path/centerline) 선택은 기존 설정대로입니다.
NC/DAC 마스킹을 EP 안에서 제거하고 PDMS 외부 곱 NC*DAC는 유지합니다.
GT의 EP=1은 baseline 정의이며 GT의 안전성이 보장된다는 뜻이 아닙니다.
raw GT 자체의 arc length를 분모로 사용하는 방식이 아니라 기존 GT rollout 비교를 유지합니다.
5m 저진행 일괄1 규칙은 제거했습니다. d_gt<=0.05m이면 모델도 <=0.05m일 때 EP=1, 그 외 EP=0.
GT path가 완전히 정지하여 기존 project_gt_path가 이동 rollout을 거부하는 조건은 아직 유지됩니다.
끝점/시간 부족/MPC 수렴 실패 정책은 변경하지 않습니다. 이들은 후속 구현 방향 검토 대상입니다.
실패 sample에 GT EP=1을 강제로 채우지 않으며 invalid/null을 유지합니다.
결과에 ep_normalization=gt_rollout_capped_ratio를 저장합니다. 이전 점수와 혼합하지 마세요.

## 설치 후 새 실행

```bash
source pdms/bin/activate
source scripts/setup_pdms_docker_env.sh nuscenes v1.0-trainval
pdms evaluate --out nuscenes_gt_baseline --limit 10 --workers 1
pdms serve --run nuscenes_gt_baseline
```

## 기존 결과의 DAC 진단 (재평가 불필요)

```bash
python tools/diagnose_dac.py --run runs/nuscenes_gt_path --token 009baef9bdf640fa89c92839bef77898
```

--run은 실제 결과 폴더로 수정하세요. gt_raw는 기록된 ego pose에 평가용 Ioniq5 footprint를 적용한 검사입니다.
이것은 원본 수집 차량 자체의 DAC가 아닙니다. gt_rollout/pred_rollout은 평가 시와 같은 검사입니다.
최초 실패 frame으로 뷰어 슬라이더를 옮겨 corners.xy 위치를 확인하세요.
contains는 경계를 제외합니다. distance_outside_m=0, on_boundary=true는 경계 접촉입니다.
진단 도구는 점수/판정/원본 데이터를 변경하지 않습니다.

이 문서의 정규화 규칙은 기존 README/GT_PATH_EP.md의 최대 진행도 및 NC/DAC 마스킹 설명을 대체합니다.
