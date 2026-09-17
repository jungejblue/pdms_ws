# Scene 시작 속도 보완 및 CPU 병렬 평가

좌표계 cache 업데이트가 적용된 저장소에서 사용합니다.

```bash
source pdms/bin/activate
source scripts/setup_pdms_docker_env.sh nuscenes v1.0-trainval
pdms inspect
pdms evaluate --out nuscenes_start_check --limit 10 --workers 1
pdms serve --run nuscenes_start_check
```

첫 샘플의 initial_speed_source가 forward_pose_difference이면 시작 경계 보완을 사용한 것입니다.
과거 0.1초가 있으면 기존 backward_pose_difference를 그대로 사용합니다.
과거 범위가 scene 시작 때문에 부족할 때만 미래 0.1초 위치 차분을 사용합니다.
현재 시각이 원본 pose 범위 밖이거나, 중간 timestamp gap, 미래 GT/객체 부족은 그대로 invalid입니다.
ETRI와 nuScenes에 같은 정책을 적용합니다. 이 정책은 오프라인 GT 기반 초기화입니다.
동일한 초기 속도를 GT 및 prediction MPC 모두에 적용하며 prediction으로 속도를 추정하지 않습니다.

## 최대 4개 worker

```bash
pdms evaluate --out nuscenes_parallel --workers 4
pdms serve --run nuscenes_parallel
```

ETRI:

```bash
source scripts/setup_pdms_docker_env.sh etri
pdms evaluate --out etri_parallel --workers 4
pdms serve --run etri_parallel
```

로컬에서는 setup_pdms_env.sh를 사용합니다. port는 7200입니다.
workers 기본값은 1이며 허용 범위는 1~4입니다. 0, 5 이상은 오류로 거부합니다.
scene마다 독립 작업을 배정하고 작업 안에서 샘플 및 MPC 시간 단계를 순차 실행합니다.
실제 worker 수는 요청값과 선택된 scene 작업 수 중 작은 값입니다.
따라서 --limit 10이 한 scene만 포함하면 --workers 4여도 병렬 속도 개선이 없습니다.

부모가 원본 metadata와 PKL을 읽어 인덱싱하고, worker에는 scene에 필요한 데이터만 전달합니다.
작업을 worker 수만큼만 제출해 대기 데이터 복제를 제한합니다. nuScenes 지도는 프로세스별로 캐시합니다.
각 worker의 BLAS/OpenMP 스레드는 1개입니다. 부모 프로세스와 파일 캐시도 메모리를 사용하므로
메모리가 부족하면 --workers 2로 줄이세요. GPU는 사용하지 않습니다.

진행 로그는 scene 작업 완료 시 모아서 출력되므로 한동안 출력이 없다가 여러 줄이 나올 수 있습니다.
샘플별 결과는 계산 직후 저장됩니다. 메인 결과 JSON/CSV는 원래 선택 token 순서로 집계됩니다.
worker 자체가 비정상 종료하면 실행을 실패로 알리며 모델 점수 실패로 위장하지 않습니다.
중단 후 이어하기는 이번 변경에 포함하지 않습니다. 중단한 run은 새 이름으로 재실행하세요.

## 결과에서 확인할 필드

sample_scores.json 또는 samples/<id>/result.json:

- initial_speed_mps: 초기 속도
- initial_speed_source: backward_pose_difference / forward_pose_difference
- initial_speed_window_s: 0.1
- initial_speed_uses_future: 미래 pose 사용 여부
- evaluation_stage: gt_mpc / prediction_mpc / metrics / complete (해당 단계에 도달한 경우)

summary.json:

- workers_requested / workers_effective
- initial_speed_policy
- forward_initial_speed_count: 초기속도 정보를 기록한 샘플 중 전방 차분 사용 개수 (후속 MPC 실패 포함)

코드 수식, MPC 초기값, timestep, submetric 정의는 변경하지 않습니다.
scene 첫 프레임을 추가 평가하므로 전체 평균은 종전의 유효 샘플 평균과 달라질 수 있습니다.
모델 비교에는 동일한 token 목록과 초기화 정책을 사용하세요.

## 단일/병렬 비교

```bash
pdms evaluate --out serial_compare --tokens /path/to/tokens.txt --workers 1
pdms evaluate --out parallel_compare --tokens /path/to/tokens.txt --workers 4
```

tokens.txt에는 최소 두 scene의 token을 한 줄씩 지정합니다.
동일 환경에서 sample_scores.json과 trajectories.npz, invalid 개수를 비교하고 실행 시간과
메모리를 측정하세요. 프로세스 시작 및 데이터 전달 비용이 있어 4배 가속을 보장하지 않습니다.

```bash
python -m pytest -q
```
