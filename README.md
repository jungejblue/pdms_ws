# pdms_ws

ETRI 원본 parquet, infos PKL, planning prediction PKL을 이용해 MPC-CLF 추종과
NC / DAC / EP / TTC / C / PDMS 평가를 수행하고 시나리오별 JSON 및 localhost 뷰어를 제공합니다.
공식 NAVSIM과는 GT reference·제어기·평가 시간이 다른 데이터셋별 개발용 지표입니다.

nuScenes도 원본 JSON metadata/map expansion + infos PKL + planning PKL로 평가할 수 있습니다.
nuScenes도 ETRI처럼 세 입력을 사용하며, prediction·infos·원본 sample token을 연결합니다.
기본 frame은 py123d의 `current_ego_rear_axle` 캐시이며 LiDAR 회전을 추가하지 않습니다.
업데이트 후 환경설정을 다시 source하고, 기존 점수는 새 출력 폴더에 재평가해야 합니다.
[nuScenes 데이터 배치·좌표계·평가 조건](docs/NUSCENES.md)을 먼저 확인하세요.
설치 후 빠른 실행:

```bash
source pdms/bin/activate
source scripts/setup_pdms_env.sh nuscenes v1.0-mini
pdms inspect
pdms evaluate --out nuscenes_check --limit 5
pdms serve --run nuscenes_check
# http://localhost:7200
```

nuScenes는 기본 IONIQ 5 가상 차량으로 평가합니다. 캐시 rear-axle 좌표를 검증해 사용하지만 실제 nuScenes 차량 치수와 일치하는 것은 아닙니다.
공식 nuScenes/NAVSIM 벤치마크 점수도 아닙니다.
아래 ETRI 명령은 그대로 사용할 수 있습니다.

## 1. 설치

```bash
git clone https://github.com/jungejblue/pdms_ws.git
cd pdms_ws
sudo apt install python3-venv
python3 -m venv pdms
source pdms/bin/activate
python -m pip install --upgrade pip
python -m pip install -e '.[viewer,test]'
python -m pip install torch --index-url https://download.pytorch.org/whl/cpu
```

## 2. 데이터 디렉터리 설정

`scripts/setup_pdms_env.sh`의 디렉터리를 실제 위치로 수정합니다.
로컬은 `$HOME/data/etri`, Docker는 `/data/etri`가 기본입니다.

```bash
export ETRI_DATA_ROOT="${DATASET_ROOT}/etri"
export ETRI_CACHE_PATH="${ETRI_DATA_ROOT}/cache"
export ETRI_PREDICTION_CACHE="${ETRI_DATA_ROOT}/result/stage2"
export PDMS_DATA_ROOT="${ETRI_DATA_ROOT}/val"
```

- `ETRI_CACHE_PATH`: infos PKL들이 있는 폴더
- `ETRI_PREDICTION_CACHE`: 같은 모델·실험의 planning PKL들이 있는 폴더
- `PDMS_DATA_ROOT`: 시나리오 폴더들이 있는 원본 parquet split 폴더. `val`은 실제 배치에 맞게 수정

원본 파일은 `<PDMS_DATA_ROOT>/<scene_token>/meta/` 또는 시나리오 바로 아래에
`ego_pose.parquet`, `hd_ego_pose.parquet`, `object.parquet`, `hd_map.parquet`가 필요합니다.
PKL 파일명을 환경변수에 지정하거나 별도 raw_data.pkl을 만들 필요가 없습니다.

## 3. 검사 → 평가 → 시각화

```bash
source pdms/bin/activate
source scripts/setup_pdms_env.sh
pdms inspect
pdms evaluate --out vad_check --limit 5
pdms serve --run vad_check
```

브라우저에서 http://localhost:7200 을 엽니다. 서버 종료는 Ctrl+C입니다.
전체 prediction을 평가하려면 다음을 사용합니다.

```bash
pdms evaluate --out vad_full
pdms serve --run vad_full
```

결과는 저장소의 `runs/vad_full`에 생성됩니다. `--out runs/vad_full`도 같은 의미이며,
절대 경로도 지원합니다. 기존 결과는 덮어쓰지 않으므로 새 실행에는 새 이름을 사용하세요.
`serve`는 저장된 결과만 읽으며 재평가하지 않습니다.

## 4. PKL 자동 탐색 규칙

지정한 폴더 바로 아래의 `.pkl`·`.pickle`을 파일명 순으로 읽습니다. 하위 폴더는 탐색하지 않습니다.
파일 이름 대신 내부 `infos` 목록 또는 `plan_results` 딕셔너리 구조로 분류합니다.
자신의 신뢰할 수 있는 데이터 폴더만 지정하세요. Pickle 로딩 자체가 코드를 실행할 수 있습니다.

- 서로 다른 token은 병합하고 동일 token·동일 내용은 한 번만 평가합니다.
- infos 중복은 평가에서 사용하는 token·scene_token·timestamp를 비교합니다.
- planning 중복은 전체 후보 배열과 command가 같아야 합니다.
- 동일 token의 값이 다르면 파일명을 표시하고 중단합니다. 최신 파일로 덮어쓰지 않습니다.
- 다른 종류의 PKL은 제외하고 `inspect`에 이유를 표시합니다.
- 읽기 실패 또는 인식된 컨테이너 구조 오류는 중단합니다.
- planning의 개별 경로 shape/command 오류는 `inspect`에서 표시하며 평가에서는 invalid로 보존합니다.
- 사용 가능한 입력이 없으면 평가를 중단합니다.

같은 token이 없어도 서로 다른 모델의 결과인지는 자동 판별할 수 없습니다.
한 prediction 폴더에는 같은 모델·같은 실험·같은 좌표/시간 규약의 결과만 두세요.
train/val을 cache에 함께 두어도 평가 대상은 기본적으로 planning에 존재하는 token입니다.
해당 token의 원본 시나리오는 PDMS_DATA_ROOT에서 접근 가능해야 합니다.

`pdms inspect`에서 selected_files, skipped_files, duplicate_tokens,
matching_ETRI_tokens, errors를 확인하세요.
평가의 `input_manifest.json`에는 선택·제외 파일, SHA-256, token별 원본 파일을 기록합니다.

특정 입력만 사용하려면 파일 또는 폴더를 직접 지정할 수 있습니다.

```bash
pdms evaluate --planning-dir /data/etri/result/model_A --out model_A
pdms evaluate --planning-pkl /data/one_prediction.pkl --out one_file
pdms inspect --infos-dir /data/etri/cache --planning-dir /data/etri/result/stage2
```

CLI 옵션 > 디렉터리 환경변수 > 이전 PDMS_RAW_PKL/PDMS_PLANNING_PKL 환경변수 순으로 적용합니다.
setup 스크립트는 이전 파일명 변수를 해제하고 경로를 다시 설정합니다. 임시 override는 source 후 지정하세요.

## 5. 결과와 평가 설정

| 파일 | 내용 |
|---|---|
| scenario_scores.json / .jsonl / .csv | 시나리오별 점수 |
| sample_scores.json / scores.csv | 샘플 점수, invalid 이유 |
| input_manifest.json | 입력 파일과 token 출처 |
| summary.json | 전체 집계 및 완료 여부 |
| config.resolved.json | 실행 설정 |
| samples/ | 뷰어용 궤적·장면·진단 |

시나리오 점수는 샘플 점수 평균입니다. invalid가 섞인 시나리오의 대표 점수는 null이며,
유효 샘플 평균은 valid_sample_mean에 별도로 남깁니다. 종료 코드 2이면 입력 오류 또는 invalid를 확인하세요.

기본 설정은 `configs/ioniq5_2023.yaml` 하나입니다.
`map_mode: approximate`는 검토된 polygon이 없을 때 centerline 폭으로 영역을 근사합니다.
검토된 `map_polygons.geojson`을 필수로 요구하려면 같은 YAML에서 `map_mode: validated`로 바꾸세요.
`representation: step_offsets`는 변위를 한 번 누적합니다. 이미 누적된 waypoint는 `relative_positions`를 사용하세요.
GT 3초, 객체 3.9초의 데이터가 필요합니다. 자세한 규격은 [DATA_FORMAT](docs/DATA_FORMAT.md)을 참고하세요.

## 6. Docker

컨테이너 안에서 활성화할 venv를 설치한 뒤 실행합니다.

```bash
source pdms/bin/activate
source scripts/setup_pdms_docker_env.sh
pdms inspect
pdms evaluate --out vad_full
pdms serve --run vad_full
```

Docker 설정의 바인딩 주소는 `0.0.0.0`, 포트는 7200입니다.
컨테이너 생성 시 `-p 127.0.0.1:7200:7200`과 데이터 폴더의 `/data` 마운트가 필요합니다.
설정 스크립트 자체는 컨테이너를 생성하지 않습니다. 호스트 브라우저에서는 http://localhost:7200 으로 접속합니다.

## 7. 데모 및 테스트

```bash
pdms demo --out demo
pdms serve --run demo/evaluation
python -m pytest -q
```

데모는 합성 예제이며 실제 모델 성능이 아닙니다.
추가 명령은 [COMMANDS](docs/COMMANDS.md), 계산 정의는 [METRIC_AND_CONTROLLER](docs/METRIC_AND_CONTROLLER.md),
출처는 [NOTICE](NOTICE.md)를 참고하세요.
