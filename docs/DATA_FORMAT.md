# 데이터 형식

## Planning PKL

```python
{
    "plan_results": {
        "SCENARIO_00000000": [prediction, command],
    }
}
```

prediction: numpy array 또는 torch.Tensor `[3,6,2]`; command: 길이3으로 flatten 가능한 one-hot `[1,1,1,3]`. command로 후보를 선택하고 GT를 이용해 best-of-three를 고르지 않습니다. 이미 선택한 `[6,2]`도 `[prediction, command]` 안에서 지원합니다.

시간은 0.5/1/1.5/2/2.5/3초입니다. 기본은 step displacement이며 cumsum 후 현재 rear-axle 기준 위치가 됩니다. 절대 world 좌표는 지원하지 않습니다. `relative_positions`는 현재 ego 기준 누적 위치입니다. 기본 x전방/y좌측, 필요하면 `axes: x_right_y_forward` 변환을 사용하세요.

py123d/VAD와 planning PKL 구조를 공유하지만, **nuScenes raw dataset adapter는 포함하지 않았습니다.** nuScenes token과 ETRI token을 임의 대응시키지 마세요.

## 기존 infos PKL

```python
{"infos": [
    {"token": "SCENARIO_00000000", "scene_token": "SCENARIO", "timestamp": 1768182874400352},
]}
```

추가 필드는 허용합니다. `token`은 유일해야 하고 planning의 key와 일치해야 합니다. `scene_token`은 디렉터리 이름이며 timestamp는 기본 us입니다. GT는 infos의 sparse waypoint가 아닌 ego_pose에서 읽습니다. infos만 있으면 지도/객체가 없으므로 `--data-root`가 필수입니다.

원본 parquet는 `--data-root`에서 직접 읽으며 새 raw PKL은 만들지 않습니다. 기존 raw/infos PKL의 추가 필드는 그대로 허용합니다. pickle은 신뢰할 수 있는 파일만 사용하세요.

## 원본 parquet 필수 컬럼

| 파일 | 컬럼 |
|---|---|
| ego_pose | timestamp, x, y, yaw |
| hd_ego_pose | timestamp, x, y, yaw |
| object | timestamp, class, obj_id, x[m], y[m], heading[rad], width[m], length[m] |
| hd_map | id, class, points |

위치는 m, 각도는 rad입니다. object에는 `class=ego` 행이 있어야 하고 hd_map에는 `class=centerline` polyline이 필요합니다. 객체의 longitudinal length를 width[m], lateral width를 length[m]로 읽는 것은 이 프로젝트에서 확인한 ETRI converter 규약입니다. 다른 converter면 config를 변경하세요.

각 source는 자신의 t0 ego pose로 local 변환합니다. 이미 ego-local로 바꾼 parquet를 원본 world 좌표처럼 입력하지 마세요. 평면 yaw 변환이며 roll/pitch를 사용하는 GT converter와는 작은 차이가 있을 수 있습니다.

## 시간 coverage

- ego_pose: t0-0.1 ~ t0+3.0초
- hd_ego_pose: t0를 얻을 수 있는 구간
- object의 ego 및 관측 frame: t0 ~ t0+3.9초
- 기본 최대 보간 gap: 0.16초

TTC는 3초 시점에서도 0.9초 더 앞을 검사합니다. 미래 데이터가 없는 test clip이나 시나리오 말미는 invalid가 될 수 있습니다. 중복 timestamp와 track 내부 큰 gap도 거부합니다.

## 지도 및 EP route

기본 approximate 모드는 centerline 폭 3.5 m를 buffer하여 DAC 영역을 만듭니다. 교차로도 단순 접속점으로 근사합니다. 정확한 도로 경계가 아니므로 점수/순위가 지도 근사에 좌우될 수 있습니다.

검증한 GeoJSON은 `SCENARIO/map_polygons.geojson`에 둡니다. 좌표는 **hd_map과 같은 planar world metres**입니다. 경위도 GeoJSON을 그대로 넣지 마세요.

- Feature `properties.role`: `drivable`, `lane`, `intersection`
- drivable/lane 필수; 교차로가 있으면 intersection도 제공
- 최상위 `properties.verified=true`: 실제 geometry 검토를 했다는 provenance

편집 template:

```bash
pdms map-template \
  --hd-map /path/to/SCENARIO/meta/hd_map.parquet \
  --out /path/to/map_polygons_draft.geojson
```

출력은 **unverified 근사 지도**입니다. 실제 도로/차선/교차로에 맞게 편집한 후 시나리오 폴더로 배치하세요. true로 바꾸는 것만으로 정확한 지도가 되지 않습니다. 정확한 이름의 map_polygons.geojson이 있으면 approximate 모드에서도 우선 읽으므로 미완성 draft는 다른 이름으로 보관하세요.

EP의 route는 두 rollout이 공유하는 하나의 누적 길이 path입니다. ID를 숫자순으로 정렬하거나 ID별 progress를 독립 계산하지 않습니다. GT는 evaluation route 선택에만 사용하고 MPC에 전달하지 않습니다. 자동 연결이 모호하면 `SCENARIO/route_ids.json`에 정확한 순서를 지정합니다.

```json
{"SCENARIO_00000000": [179,175,206,222,201,204]}
```

## JSON 집계

각 샘플은 `NC*DAC*(5*EP+5*TTC+2*C)/12`로 계산합니다. 시나리오 PDMS는 그 샘플 PDMS들의 평균입니다. incomplete 시나리오의 대표 점수는 null이며 valid-only 평균은 진단 필드에 따로 있습니다. global summary도 incomplete일 때 대표 macro mean은 null입니다.

지도나 GT baseline이 실패해 점수 0이 나온 경우는 계산 완료(valid)입니다. 데이터/solver 오류로 계산 자체가 실패한 경우가 invalid입니다. 알 수 없는 prediction token은 `__unmatched__`에 모아 기록합니다.

## 디렉터리 입력

`ETRI_CACHE_PATH` 및 `ETRI_PREDICTION_CACHE`는 파일명이 아닌 폴더를 지정합니다.
바로 아래 `.pkl`/`.pickle`을 정렬하여 읽고 infos/plan_results 구조를 확인합니다.
재귀 탐색은 하지 않으며 단일 파일 입력도 계속 지원합니다.
동일 token·동일 값은 중복 제거하고, 충돌은 원본 파일명과 함께 오류로 알립니다.
infos에서는 평가에 사용하는 token·scene_token·timestamp만 비교합니다.
planning은 전체 후보와 command를 비교하며, 서로 다른 모델 결과는 별도 폴더로 나누세요.
`input_manifest.json`에 사용한 파일 hash와 token 출처를 기록합니다.
