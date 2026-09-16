# pdms_ws

ETRI raw data와 VAD planning PKL을 읽어 **GT baseline PDMS 및 NC / DAC / EP / TTC / C**를 계산하고, **시나리오별 JSON**과 **localhost Viser 뷰어**를 제공합니다.

- 모델·GT: 동일 MPC-CLF + 사용자 제공 2023 IONIQ 5 kinematic bicycle model
- 모델 출력: 2 Hz, 3초, 6개 waypoint → 10 Hz 보간
- 출력: `scenario_scores.json`에 **시나리오마다 한 객체·한 줄**
- 환경: Python 3.10 이상, **`pdms`라는 이름의 venv**, CPU 평가
- 입력: 기존 raw/infos PKL + planning PKL + 원본 ETRI parquet 폴더. 별도 raw PKL 생성·변환 없음

이 지표는 `ETRI-PDMS-GT-MPC`입니다. 공식 NAVSIM과 horizon·controller·reference·데이터 adapter가 다릅니다. 기본 지도 설정은 centerline 폭을 사용한 **개발용 근사**이며, 최종 비교에는 검증된 polygon을 사용하세요.

## 1. 설치

GitHub의 이 저장소를 clone하거나 소스 ZIP을 해제한 후 `pdms_ws` 디렉터리에서 실행합니다.

```bash
# 게시된 저장소 소유자의 GitHub ID로 변경
export PDMS_GITHUB_OWNER="REPOSITORY_OWNER"
git clone "https://github.com/${PDMS_GITHUB_OWNER}/pdms_ws.git"
cd pdms_ws
```

```bash
cd /path/to/pdms_ws

# Ubuntu에서 venv 모듈이 없는 경우 한 번 설치
sudo apt install python3-venv

python3 -m venv pdms
source pdms/bin/activate

python -m pip install --upgrade pip
python -m pip install -e '.[viewer,test]'

# torch.Tensor가 저장된 실제 planning PKL을 읽을 때 필요
python -m pip install torch --index-url https://download.pytorch.org/whl/cpu

export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
```

새 터미널에서는 다음을 실행합니다.

```bash
cd /path/to/pdms_ws
source pdms/bin/activate
export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
```

종료:

```bash
deactivate
```

## 2. 데이터 없이 데모 실행

```bash
pdms demo --out runs/demo
python -m json.tool runs/demo/evaluation/scenario_scores.json
pdms serve --run runs/demo/evaluation --port 7200
```

브라우저: **http://localhost:7200**

서버 터미널을 켜 둔 상태로 사용하며 종료는 **Ctrl+C**입니다. 데모는 합성 데이터이고 실제 VAD 성능이 아닙니다. 합성 infos/planning PKL과 parquet를 생성한 뒤 원본 파일을 직접 읽는 평가 경로로 실행합니다.

```text
runs/demo/infos.pkl
runs/demo/data/synthetic_straight/*.parquet
runs/demo/predictions.pkl
runs/demo/evaluation/scenario_scores.json
```

이미 결과가 있는 출력 디렉터리는 덮어쓰지 않습니다. 재실행할 때 `runs/demo_02`처럼 새 경로를 지정하세요.

## 3. 실제 데이터 준비

infos PKL의 `scene_token`과 시나리오 폴더 이름이 같아야 합니다. 필수 파일은 다음 위치 중 하나에 둡니다.

```text
/path/to/ETRI/val/SCENARIO/meta/ego_pose.parquet
/path/to/ETRI/val/SCENARIO/meta/hd_ego_pose.parquet
/path/to/ETRI/val/SCENARIO/meta/object.parquet
/path/to/ETRI/val/SCENARIO/meta/hd_map.parquet
```

`meta/` 없이 `SCENARIO/` 바로 아래에 두어도 됩니다. 지도 보완 파일은 시나리오 바로 아래에 둡니다.

```text
/path/to/ETRI/val/SCENARIO/map_polygons.geojson
/path/to/ETRI/val/SCENARIO/route_ids.json
```

카메라/LiDAR 원본, py123d Arrow 변환본, 학습 weight는 평가 입력에 필요하지 않습니다. 미래 GT와 객체가 공개된 train/validation 구간을 사용하세요. timestamp 단위, 필수 컬럼, PKL 스키마는 [데이터 형식](docs/DATA_FORMAT.md)에 있습니다.

기존 raw data PKL(예: `etri_infos_temporal_val.pkl`)을 그대로 `--raw-pkl`에 지정하고, 원본 parquet가 있는 split 폴더를 `--data-root`에 지정합니다. 파일을 복사하거나 별도 `raw_data.pkl`로 묶지 않습니다. 원본 데이터는 읽기만 하며 결과는 `--out`에 저장합니다.

`--raw-pkl`은 기존 ETRI infos 구조의 파일을 받습니다. PKL 이름만으로 임의의 데이터셋 스키마를 자동 해석하지는 않습니다.

## 4. PKL 확인 → 평가 → localhost 시각화

### 입력 형식 및 token 확인

```bash
pdms inspect \
  --raw-pkl /path/to/etri_infos_temporal_val.pkl \
  --planning-pkl /path/to/planning.pkl \
  --config configs/ioniq5_2023.yaml
```

`errors=[]`, `unmatched_prediction_tokens=0`인지 확인합니다. 이 명령은 PKL 형식·token을 검사하며, 전체 parquet 시간 범위나 MPC 실행까지 검사하지는 않습니다.

### 5개 샘플로 먼저 확인

```bash
pdms evaluate \
  --raw-pkl /path/to/etri_infos_temporal_val.pkl \
  --data-root /path/to/ETRI/val \
  --planning-pkl /path/to/planning.pkl \
  --config configs/ioniq5_2023.yaml \
  --out runs/vad_check \
  --limit 5

pdms serve --run runs/vad_check --port 7200
```

브라우저에서 http://localhost:7200 을 열어 GT·예측·객체 위치와 지도 정렬을 확인하세요.

### 전체 prediction 평가

```bash
pdms evaluate \
  --raw-pkl /path/to/etri_infos_temporal_val.pkl \
  --data-root /path/to/ETRI/val \
  --planning-pkl /path/to/planning.pkl \
  --config configs/ioniq5_2023.yaml \
  --out runs/vad_full

pdms serve --run runs/vad_full --port 7201
```

모델 비교에는 같은 평가 token 목록을 사용합니다. 각 줄에 exact token 하나를 적으세요.

```bash
pdms evaluate \
  --raw-pkl /path/to/etri_infos_temporal_val.pkl \
  --data-root /path/to/ETRI/val \
  --planning-pkl /path/to/planning.pkl \
  --tokens /path/to/eval_tokens.txt \
  --config configs/ioniq5_2023.yaml \
  --out runs/vad_manifest
```

`--tokens`가 없으면 planning PKL에 존재하는 token만 평가합니다. `--tokens`로 요청한 예측이 없으면 해당 샘플을 invalid로 남깁니다. 평가 대상 token은 `evaluated_tokens.txt`에 저장됩니다.

## 5. 시나리오별 JSON 읽기

```bash
python -m json.tool runs/vad_full/scenario_scores.json
```

`scenario_scores.json`은 JSON 배열이고, **각 시나리오가 한 줄의 객체**입니다. 같은 내용을 JSON Lines 형식인 `scenario_scores.jsonl`과 CSV로도 저장합니다.

```json
[
  {"scenario_id":"SCENARIO_A","status":"complete","num_samples":10,"num_valid":10,"num_invalid":0,"coverage":1.0,"NC":1.0,"DAC":0.9,"EP":0.8,"TTC":1.0,"C":0.9,"PDMS":0.82}
]
```

위 숫자는 형식을 보여 주는 예시입니다. 각 점수는 해당 시나리오의 **샘플 점수 평균**이며, 평균 NC/DAC/EP 등을 다시 공식에 넣어 PDMS를 계산하지 않습니다. 실제 출력에는 `valid_sample_mean`, 지도 품질, GT baseline 실패 횟수, invalid 이유도 포함됩니다.

| 파일 | 내용 |
|---|---|
| `scenario_scores.json` | 시나리오별 한 객체·한 줄의 JSON 배열 |
| `scenario_scores.jsonl` | 시나리오별 한 줄의 JSON Lines |
| `scenario_scores.csv` | 같은 집계의 표 형태 |
| `sample_scores.json`, `scores.csv` | 모든 요청 샘플의 점수 및 invalid 이유 |
| `summary.json` | 전체 집계·유효 개수·설정 hash·공식 지표와의 구분 |
| `samples/<id>/` | localhost가 읽는 NPZ·scene·진단 JSON |

샘플이 하나라도 invalid인 시나리오는 `status=partial/invalid`, 대표 점수 `NC`~`PDMS`는 `null`입니다. 유효 샘플만의 평균은 `valid_sample_mean`에 별도로 남깁니다. 실패 샘플을 조용히 제외한 숫자를 완전한 시나리오 점수로 보고하지 않습니다.

평가 종료 코드 0은 모든 요청 샘플 계산 완료, 2는 입력 오류 또는 invalid 샘플 발생입니다. 출력이 생성됐다면 `sample_scores.json`의 `invalid_reason`을 확인하세요. 점수가 0인 정상 계산 샘플은 invalid가 아닙니다.

## 6. localhost 화면 조작

- **Sample / Previous / Next:** 샘플 선택·이동
- **Token / ID:** token 검색 및 이동
- **Frame / Play / Loop / Speed:** 시간·재생·반복·배속
- **Layers / legend:** 지도, EP route, VAD/GT 경로·차량, reference, 객체 표시
- **Scores and diagnostics:** 하위 점수, 추종 오차, 지도 품질, 이벤트
- **Control charts:** 속도·가속도·조향 그래프
- **Reset BEV camera:** 위에서 보는 시점으로 복귀

주황은 VAD, 초록은 GT, 파랑은 EP route입니다. 기본 포트 7201을 변경하려면 `--port`를 지정하세요. 여러 브라우저가 연결되면 샘플/시간/토글은 공유됩니다.

원격 서버에서 외부 인터페이스로 접속할 때:

```bash
pdms serve --run runs/vad_full --host 0.0.0.0 --port 7200
```

Docker는 `-p 7200:7200` 매핑도 필요합니다. SSH 터널은 [추가 명령](docs/COMMANDS.md)을 참고하세요.

## 7. 평가 전 설정 확인

| 설정 | 기본값 / 확인 사항 |
|---|---|
| `representation` | `step_offsets`: evaluator에서 cumsum 1회. 이미 누적 좌표이면 `relative_positions` |
| `axes` | `x_forward_y_left`: 현재 rear axle 좌표계, m 단위 |
| `raw_timestamp_unit` | parquet: `ms` |
| `info_timestamp_unit` | infos: `us` |
| `map_mode` | `approximate`: 개발용 centerline buffer |

최종 지도 평가에는 검토한 `map_polygons.geojson`을 준비하고 아래 설정을 사용하세요.

```bash
pdms evaluate \
  --raw-pkl /path/to/etri_infos_temporal_val.pkl \
  --data-root /path/to/ETRI/val \
  --planning-pkl /path/to/planning.pkl \
  --config configs/ioniq5_2023_validated.yaml \
  --out runs/vad_validated
```

지도 파일을 수정했다면 새 `--out` 경로로 다시 평가하세요. 지도 작성과 route 연결은 [데이터 형식](docs/DATA_FORMAT.md)을 참고하세요.

## 8. 테스트

```bash
python -m pytest -q
```

- [데이터 형식과 좌표계](docs/DATA_FORMAT.md)
- [컨트롤러·지표 정의 및 제한](docs/METRIC_AND_CONTROLLER.md)
- [추가 실행 명령](docs/COMMANDS.md)
- [GitHub 게시 명령](docs/PUBLISH.md)
- [출처 및 라이선스](NOTICE.md)
