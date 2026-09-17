# nuScenes 평가 및 localhost 시각화

## 1. 설치

저장소 루트에서 실행합니다. 업데이트 패키지 사용자는 먼저 패키지의 APPLY.md를 따릅니다.

```bash
python3 -m venv pdms
source pdms/bin/activate
python -m pip install --upgrade pip
python -m pip install -e '.[viewer,test]'
# torch Tensor가 포함된 planning PKL에만 필요
python -m pip install torch --index-url https://download.pytorch.org/whl/cpu
```

추가 nuScenes SDK 설치는 필요하지 않습니다. JSON metadata와 map expansion을 직접 읽습니다.

## 2. 데이터 준비

nuScenes 이용 조건에 따라 `v1.0-mini` 또는 `v1.0-trainval` metadata와
map expansion의 vector JSON(예: v1.3)을 준비합니다. annotation이 없는 test는 지원하지 않습니다.
카메라 이미지·LiDAR binary·sweeps binary는 필요하지 않으며, 원본 JSON 테이블은 필요합니다.

기본 로컬 배치(기존 데이터 위치가 다르면 setup script의 NUSCENES_DATA_ROOT를 수정):

```text
~/data/nuscenes/v1.0-mini/
  v1.0-mini/
    sample.json
    scene.json
    sample_data.json
    ego_pose.json
    calibrated_sensor.json
    sensor.json
    log.json
    sample_annotation.json
    instance.json
    category.json
  maps/expansion/
    boston-seaport.json
    singapore-onenorth.json
    singapore-hollandvillage.json
    singapore-queenstown.json
  cache/
    <원하는_infos_이름>.pkl
  result/stage2/
    <원하는_이름>.pkl
```

`NUSCENES_DATA_ROOT`는 버전 폴더와 maps 폴더를 포함하는 상위 폴더입니다.
위 경로에서 `v1.0-mini/v1.0-mini`가 반복되는 것은 데이터셋별 루트와 원본 metadata 폴더를 구분하기 때문입니다.
지도 위치만 다른 경우 `NUSCENES_MAP_ROOT`를 수정합니다. 해당 경로 아래에 `maps/expansion/`이 있어야 합니다.
`map_polygons.geojson`은 사용하지 않습니다. nuScenes infos PKL은 `cache/`에 배치합니다.
환경변수 `NUSCENES_CACHE_PATH`로 해당 폴더를 지정합니다. infos는 `{"infos": [...]}` 또는
info 목록 형식이며 각 항목에 `token`, microsecond 단위 `timestamp`가 필요합니다.
원본 sample token과 일치해야 하며 timestamp도 원본과 일치하는지 확인합니다.
scene_token이 없는 VAD/nuScenes infos도 지원하고 시나리오는 원본 sample.json에서 구합니다.

planning PKL은 기존 `plan_results[token] = [prediction, command]` 형식입니다.
선택된 경로 `[6,2]` 또는 command로 선택할 3개 후보 `[3,6,2]`를 지원합니다.
키는 frame 번호나 scene token이 아니라 원본 **sample token**이어야 합니다.
같은 모델·실험의 PKL들만 result/stage2에 넣습니다. 최상위 폴더의 PKL들을 스키마로 선택·병합하며,
서로 다른 예측이 같은 token에 존재하면 충돌로 중단합니다. 일부 scene의 prediction만 있어도 됩니다.

## 3. 실행

```bash
cd "$HOME/pdms_ws"  # 실제 저장소 위치
source pdms/bin/activate
source scripts/setup_pdms_env.sh nuscenes v1.0-mini

pdms inspect
pdms evaluate --out nuscenes_check --limit 5
pdms serve --run nuscenes_check
```

브라우저에서 http://localhost:7200 에 접속합니다. 종료는 Ctrl+C입니다.
소규모 결과의 GT·예측·지도 좌표 정렬과 invalid 원인을 확인한 후 전체 평가합니다.

```bash
pdms evaluate --out nuscenes_full
pdms serve --run nuscenes_full
```

trainval metadata를 사용할 때:

```bash
source scripts/setup_pdms_env.sh nuscenes v1.0-trainval
pdms inspect
pdms evaluate --out nuscenes_trainval
```

Docker 내부에서는 다음을 사용합니다. 데이터 루트는 `/data/nuscenes/<version>`이며
컨테이너 실행 시 호스트 7200과 컨테이너 7200을 연결해야 합니다(`-p 7200:7200`).

```bash
source scripts/setup_pdms_docker_env.sh nuscenes v1.0-mini
pdms evaluate --out nuscenes_check --limit 5
pdms serve --run nuscenes_check
```

ETRI로 돌아가기:

```bash
source scripts/setup_pdms_env.sh etri
pdms evaluate --out vad_full
```

환경변수를 쓰지 않고 명시할 수도 있습니다:

```bash
pdms evaluate --dataset nuscenes \
  --data-root /data/nuscenes/v1.0-mini \
  --infos-dir /data/nuscenes/v1.0-mini/cache \
  --planning-dir /data/nuscenes/v1.0-mini/result/stage2 \
  --nuscenes-version v1.0-mini --prediction-frame lidar \
  --out nuscenes_full
```

## 4. 좌표계와 점수의 의미

- 기본은 공식 VAD converter의 **현재 LIDAR_TOP 좌표축·LiDAR 원점 기준** 미래 경로입니다.
  0.5초 간격 6개 step displacement를 한 번 누적하고 calibration 회전으로 ego 축으로 변환합니다.
  LiDAR BEV의 z는 0으로 가정합니다. 이미 누적된 좌표라면 공통 YAML의
  `representation: relative_positions`로 바꿉니다.
- 자체 converter가 **ego 원점·x 전방/y 좌측** 경로를 출력하면
  `--prediction-frame ego` 또는 `export NUSCENES_PREDICTION_FRAME=ego`를 사용합니다.
  nuScenes에서는 공통 YAML의 `axes` 대신 이 frame 선택과 sensor calibration을 사용합니다.
- GT는 같은 anchor의 미래 원본 pose에서 구합니다. 센서 원점과 실제 rear axle 사이 오프셋은 복원하지 않습니다.
  **선택한 LiDAR/ego 원점에 가상 차량 rear axle을 놓는 근사**입니다. 기본 차량은 공통 YAML의 IONIQ 5입니다.
  실제 nuScenes ego footprint가 아니며 이 가정은 충돌·DAC·추종 점수에 영향을 줍니다.
- GT·모델에 같은 MPC-CLF와 가상 차량을 적용합니다. NC/DAC/EP/TTC/C 및 PDMS는
  기존 GT-reference 구현을 사용합니다. 공식 nuScenes 평가 또는 공식 NAVSIM PDMS와 직접 비교하지 마세요.
- map expansion의 drivable_area/lane/lane_connector/intersection polygon을 사용합니다.
  `map_mode`는 ETRI 지도용 옵션이며 nuScenes에서는 항상 map expansion이 필요합니다.
  누락 시 centerline 폭으로 대체하지 않습니다. 결과 `map_quality`는 `nuscenes_map_expansion`입니다.
- EP route는 미래 GT로 선택한 lane centerline과 공식 connectivity에서 구성합니다.
  lane change로 연결 불가, 경로 자기교차, 모호한 분기, 짧은 route 등은 invalid가 될 수 있습니다.
  현재 nuScenes에는 수동 route_ids.json 보정 기능이 없습니다.
- 객체 annotation(통상 2 Hz)은 위치·크기와 unwrap yaw를 10 Hz로 선형 보간합니다.
  관측 track의 처음/마지막 바깥으로는 외삽하지 않습니다. 따라서 가려지거나 등장·퇴장한 객체의
  미관측 구간을 재구성하지 않으며 NC/TTC에 이 한계가 있습니다.
- pose는 과거 0.1초부터 미래 3초, 객체 annotation 시간 범위는 미래 3.9초까지 필요합니다.
  마지막 프레임들은 invalid일 수 있습니다. pose 보간 gap 기본 0.16초, annotation gap 0.75초입니다.
  이 두 값은 YAML의 `nuscenes_pose_max_gap_s`, `nuscenes_annotation_max_gap_s`로 조정할 수 있으나
  실제 누락 구간을 허용하면 정확도가 떨어집니다.

## 5. 결과 확인

| 파일 | 내용 |
|---|---|
| runs/nuscenes_full/scenario_scores.json | scene token마다 하나의 JSON 항목, 유효 샘플 평균 및 실패 개수 |
| runs/nuscenes_full/sample_scores.json | sample별 점수·invalid 원인·scene_name·anchor 가정 |
| runs/nuscenes_full/summary.json | 집계·평가 가정·완료 여부 |
| runs/nuscenes_full/input_manifest.json | prediction/원본 metadata/map 파일과 hash |
| runs/nuscenes_full/samples/ | localhost 뷰어용 저장된 경로·지도·객체 |

prediction에 있는 token만 평가하고 infos PKL 또는 원본 metadata와 연결하지 못하거나 미래 데이터가 부족한 token은
invalid로 남깁니다. infos/raw에만 있고 prediction에 없는 sample을 자동으로 채우지 않습니다.
뷰어는 저장된 유효 샘플의 경로를 표시하며 invalid 항목은 실패 이유를 확인하는 용도입니다.
부분 prediction으로 얻은 scene 평균은 **예측이 존재하고 평가에 성공한 샘플**의 평균입니다.
시나리오 전체 데이터셋 성능으로 해석하지 마세요. 하나라도 invalid면 CLI 종료 코드 2가 반환되지만 결과는 저장됩니다.

## 6. 검증 및 참고

```bash
python -m pytest -q
```

합성 nuScenes 스키마 데이터로 좌표 회전, GT, 객체 크기, 시간 보간·경계,
map expansion, end-to-end 점수·뷰어 입력 및 기존 ETRI 회귀 테스트를 검증했습니다.
실제 nuScenes 데이터에서 점수의 타당성·runtime·지도 연결 성공률은 별도 검증이 필요합니다.

- [nuScenes 원본 스키마](https://github.com/nutonomy/nuscenes-devkit/blob/master/docs/schema_nuscenes.md)
- [nuScenes map API](https://github.com/nutonomy/nuscenes-devkit/blob/master/python-sdk/nuscenes/map_expansion/map_api.py)
- [VAD nuScenes converter](https://github.com/hustvl/VAD/blob/main/tools/data_converter/vad_nuscenes_converter.py)
