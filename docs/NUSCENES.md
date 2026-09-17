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
  --nuscenes-version v1.0-mini --prediction-frame cache \
  --out nuscenes_full
```

## 4. py123d 좌표계 규약 (기본 cache 모드)

기본 `NUSCENES_PREDICTION_FRAME=cache`는 첨부 py123d의 prediction parser와 같은 규약입니다.
infos 각 항목에 다음 metadata와 pose가 필요합니다.

```python
conversion_meta = {
    "coordinate_frame": "current_ego_rear_axle",
    "coordinate_axes": "x_forward_y_left_z_up",
    "ego_origin": "rear_axle_center",
    "quaternion_order": "wxyz",  # 생략 가능, 있으면 wxyz만 허용
}
# ego2global_translation: XYZ
# ego2global_rotation: wxyz quaternion 또는 3x3/길이9 rotation matrix
```

- command 선택 → displacement를 한 번 cumsum → rear-axle 현재 위치 [0,0]에서 시작합니다.
- LiDAR calibration 회전을 prediction에 적용하지 않습니다. 임의의 90도 보정도 하지 않습니다.
- py123d처럼 map_ego2global pose가 있으면 우선하고, 없으면 ego2global pose를 사용합니다.
- py123d의 global XYZ 결과를 PDMS의 현재 yaw 기준 평면 XY로 투영합니다. pitch/roll이 있을 때도
  단순히 XY 숫자를 복사하지 않고 cache pose 회전을 거친 global XY와 일치시킵니다.
- map/objects는 같은 global 기준의 원본 geometry를 같은 평가 local frame으로 옮깁니다.
- GT는 원본 ego_pose의 미래 rear-axle 위치에서 구합니다. 현재 cache pose와 원본 keyframe ego pose의
  차이가 0.05 m 또는 1도를 넘으면 invalid로 중단합니다. 두 소스가 별도 정합 좌표계라면
  명시적인 registration 구현이 필요하며 임의 정렬하거나 tolerance를 늘려 감추지 않습니다.
- metadata 누락/불일치는 inspect에서 오류로 표시합니다. 검사 우회를 위해 metadata를 임의로
  덧붙이지 말고 현재 converter가 만든 infos인지 확인하세요.
- 기본 cache 모드는 py123d와 같이 `representation: step_offsets`만 허용합니다.
  이미 누적된 다른 출력은 별도의 명시적인 입력 규약이 필요합니다.
- 구형 metadata 없는 파일만 `--prediction-frame lidar` 또는 `ego`를 명시해 기존 모드로 읽을 수 있습니다.
  metadata가 있는 파일에 legacy 모드를 선택하면 충돌로 중단합니다.

차량 치수와 제어기는 여전히 기존 YAML의 IONIQ 5입니다. 좌표 원점 일치가 실제 nuScenes 차량
치수까지 일치한다는 뜻은 아니며, 공식 nuScenes/NAVSIM 점수는 아닙니다.
map expansion이 항상 필요하고 ETRI용 map_mode로 이를 대체하지 않습니다.
EP route 연결 불가·길이 부족은 별도 invalid 사유입니다. nuScenes에서 route_ids.json은 읽지 않습니다.
객체 annotation은 관측 구간 안에서만 선형 보간하며 외삽하지 않습니다.
과거 pose 0.1초, 미래 GT 3초, 객체 시간 범위 3.9초 조건은 유지합니다.

기존 잘못된 frame으로 산출한 점수는 viewer 재실행만으로 고쳐지지 않습니다.
새 출력 이름으로 반드시 재평가하세요.

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
