# 구현과 MPC-CLF 코드 설명

## 첨부 원본의 역할

이 구현은 프로젝트에서 제공한 MPC-CLF 코드를 참고했습니다. 원본 standalone 예제는 이 공개 저장소에 포함하지 않습니다. executable 평가 경로는 `src/etri_pdms/tracking.py`의 적응 구현입니다. 원본 파일을 그대로 import하는 wrapper는 아니며, 아래 동역학과 single-CLF 제약을 보존해서 시간 조건이 있는 평가용으로 옮겼습니다.

| 원본 요소 | 기능 | 평가 구현 |
|---|---|---|
| `Vehicle` | 차체 제원/조향 제한, 운동 업데이트 | 2023 IONIQ 5 configuration + independent world rollout |
| `RefPathGenerator` | 직선/곡선 생성, yaw/곡률/누적 길이, world↔Frenet | 주어진 모델/GT reference에서 `SpatialPath` 생성 |
| `MPCControllerFrenetSingleCLF` | CasADi/Ipopt finite-horizon 최적화 | `FrenetCLF` |
| `s, d, epsi` | 경로 진행 거리, 횡오차, 경로 tangent 대비 heading 오차 | 같은 정의에 속도 v 상태 추가 |
| `delta, v` | 조향 및 속도를 직접 선택 | `delta, a`를 선택하고 v를 동역학으로 갱신 |
| `V=wd*d²+we*epsi²` | 횡방향/방향 오차 크기 | 같은 single-CLF |
| `V_next <= (1-c*dt)V + slack` | 오차 감소를 유도하는 완화 제약 | 동일; slack 제곱 비용, 기록 |
| 원본 main/animation | 5 m/s 직선 예제와 실시간 drawing | CLI, token별 평가, standalone HTML/PNG |

CLF는 장애물 회피나 안정성의 무조건 보장이 아닙니다. slack을 허용하므로 infeasible reference에서도 최적화가 가능하게 만들지만, 오차 감소 조건을 얼마나 완화했는지 확인해야 합니다. 원본 main의 `clf_rate=0`은 지수적 감소를 요구하지 않습니다. 본 기본값은 0.5이고 slack penalty는 1000입니다. 두 값은 개발용 설정이며 별도 calibration split으로 고정해야 합니다.

원본 default `v_min=.2`, 초기 `v_prev=0`, `accel_max=1`, `dt=.1`은 첫 속도 조건이 `.2 이상`과 `.1 이하`로 충돌할 수 있습니다. 본 구현은 v_min=0, 실제 ego 속도 초기화, acceleration state update로 이 문제를 피합니다.

원본의 `q_s=0`은 주로 공간 경로 추종이며 예측점 도착시각을 제대로 평가하지 못합니다. 본 구현은 timestamp별 s_ref/v_ref와 `q_s=2`를 사용합니다. 모델 속도 프로파일을 GT 속도로 교체하지 않습니다.

## 동역학

Frenet prediction:

\[
\dot{s}=v\cos(e_\psi)/(1-\kappa(s)d),\quad
\dot{d}=v\sin(e_\psi),\quad
\dot{e}_\psi=v\tan\delta/L-\kappa(s)\dot{s},\quad
\dot{v}=a
\]

`1-kappa*d >= .2`를 제약합니다. 이 좌표계가 특이해지는 큰 오차/급곡선 입력은 solver failure가 될 수 있습니다. 실패를 GT 경로로 교체하거나 정상 결과로 숨기지 않고 invalid로 남깁니다.

실제 scorer에 전달되는 rollout은 world rear-axle state `[x,y,yaw,v]`를 explicit Euler로 갱신합니다. 다음 제어 step에서 현재 world state를 reference path에 재투영합니다. MPC의 예측 Frenet 좌표를 그대로 world 위치로 복원해 차량을 경로에 붙이지 않습니다. Frenet/world 이산화 차이는 존재하며 tracking RMSE와 slack으로 관측할 수 있습니다.

3초 말미에는 `N=min(config.horizon, remaining_steps)`로 horizon을 줄입니다. 끝점 반복으로 억지 정지를 만들지 않습니다. 공간 path의 짧은 접선 연장은 projector/곡률 정의에만 사용하며 새로운 미래 waypoint는 만들지 않습니다.

초기 속도는 ego_pose의 직전 0.1초 이동 거리로 추정합니다. 초기 steering은 양쪽 모두 0으로 둡니다. 실제 steering/steering rate sensor를 제공하는 데이터로 확장할 때는 양쪽 controller에 같은 측정값을 적용해야 합니다. 초기 steering=0 가정의 영향은 주로 급회전 시작에서 발생할 수 있습니다.

## 차량 제원

| 항목 | 값 |
|---|---:|
| Length / Width / Height | 4.635 / 1.892 / 2.434 m |
| Wheelbase | 3.000 m |
| Front / Rear overhang | 0.845 / 0.790 m |
| Rear axle → front / rear bumper | 3.845 / 0.790 m |
| Rear axle → body center | 1.5275 m |
| Max equivalent bicycle steering | ±40° |
| Minimum turning radius metadata | 5.87 m |

사용자가 제공한 2023 수치를 사용하며 2021 값으로 바꾸지 않았습니다. 높이는 BEV 점수에 사용하지 않습니다. 40°는 equivalent single front road-wheel angle로 해석한 **모델 가정**입니다. 최소 회전 반경 5.87 m는 측정 기준이 불명확하므로 metadata로 보관하며 추가 curvature 제한으로 중복 적용하지 않습니다. 전폭은 mirror 포함/제외를 따로 추정하지 않습니다.

가속도 -4~2 m/s², 최고 속도40 m/s, 조향속도30°/s는 실제 현대 공식 보증값이 아닌 개발용 controller 설정입니다. 모델 비교 전에 별도 calibration split에서 확정하세요.

## 공식 PDMS와의 차이

집계 및 EP는 NAVSIM v1.1 소스의 의미를 따릅니다. comfort 필터는 해당 버전 원본이며 import만 로컬 StateIndex로 변경했습니다. 그러나 전체 scorer를 공식 nuPlan map/object 클래스에서 직접 실행한 것은 아닙니다.

- 3초 horizon, MPC-CLF와 IONIQ 5를 사용합니다.
- baseline은 GT의 MPC rollout입니다. PDM-Closed가 아닙니다.
- ETRI timestamps를 resample하며 객체 속도는 track 위치 차분으로 구합니다. singleton track 속도는 0으로 가정합니다.
- 미래 track 생존 구간 바깥은 객체를 생성하지 않습니다. track 내부 missing-frame gap은 invalid 처리합니다. 데이터의 탐지 누락까지 복구하는 tracker는 포함하지 않았습니다.
- 초기 overlap은 NC collision classifier에 통과시킵니다. 이전 시점 collision history를 가진 NAVSIM observation의 collided-track cache를 재현하지는 않습니다.
- `Car/Vehicle/Pedestrian/Cyclist/Bicycle/Motorcycle/Bus/Truck`은 agent, 나머지는 static 취급합니다. 다른 class vocabulary면 `metrics.AGENTS`를 수정해야 합니다.
- TTC intersection/NC multilane 조건의 정확도는 lane/intersection polygon 품질에 의존합니다.
- acceleration은 rollout control의 longitudinal a 및 `v² tan(delta)/L` lateral 값입니다. 첫 acceleration history와 steering actuator lag, slip, dynamic bicycle 효과는 모델링하지 않습니다.
- controller 초기 steering 0, 전진 전용, tire slip 없는 저속/일반 도로 kinematic 모델입니다. reverse/high-slip 평가용은 아닙니다.
- route는 GT로 선택한 evaluation-only oracle입니다. branch 정보가 없다는 이유로 모델별로 다른 route를 고르지 않습니다.

정확한 공식 NAVSIM 결과라는 명칭 대신 `ETRI-PDMS-GT-MPC-v0.1`과 config hash를 보고하세요. 지도는 `approximate_centerline_buffer` 또는 `validated_geojson`으로 표시합니다. validated는 사용자가 검토한 map annotation의 provenance를 뜻하며 공식 NAVSIM 동등성을 인증하지 않습니다.

## 소스 구성

- `prediction.py`: trusted CPU PKL reader, command 선택, cumsum/축 계약
- `data.py`: infos/token, timestamps, 3개 pose source, native GT, 객체 replay
- `geometry.py`: footprint, map adapter, ordered route stitching
- `tracking.py`: PCHIP, Frenet MPC-CLF, bicycle rollout
- `metrics.py`, `comfort.py`: NC/DAC/EP/TTC/C, pair normalization
- `evaluator.py`: 샘플 평가, invalid 기록, aggregation
- `visualization.py`: standalone Plotly 시간 슬라이더, PNG, index report
- `cli.py`, `demo.py`, `map_template.py`: 명령행/합성 데이터/지도 편집 template

ETRI pose 변환은 planar yaw를 사용합니다. 원본 converter가 roll/pitch까지 적용해 GT offset을 만들었다면 raw ADE/FDE에 작지만 비영인 차이가 있을 수 있습니다. Bicycle rollout과 BEV score는 평면 모델 기준입니다.
