# GT 경로 EP

## 동작

- Config 기본 `ep_reference: gt_path`, `ep_gt_path_seconds: 10.0` (허용 3~60초).
- CLI `--ep-reference centerline`으로 이전 EP 기준선을 선택할 수 있습니다.
- 원본 ego pose를 현재 local frame으로 변환하고, 평가 차량의 center_offset과 각 시점 yaw로 차량 중심 궤적을 만듭니다. 모델/GT rollout에도 같은 차량 중심을 적용합니다.
- GT 기준선은 원본 기록에서 0.1초 간격으로 최대 10초까지 읽습니다. 동일 좌표를 제거하며, 미래 pose gap/기록 끝에서 연장을 중단합니다. 3초 이내 coverage 실패는 그대로 invalid입니다. GT에 미리 주어진 미래 정보는 평가 기준선에만 사용하며 MPC reference는 기존 3초 경로입니다.
- 지도 polygon과 객체는 기존대로 읽습니다. nuScenes GT 모드에서는 choose_route를 호출하지 않습니다. ETRI 근사 지도 모드는 여전히 centerline으로 drivable을 구성합니다. GT 모드에서 route_ids.json은 EP에 사용하지 않습니다.
- GT/model 모두 하나의 기준선에 투영합니다. 시간 순서대로 이전 arc 위치 주변(차량 중심 이동 거리의 2배 + 0.1m)에서 최근접 투영 후보를 선택합니다. 시작 후보는 arc 0에서 0.5m 이내입니다. 경로 교차에서 먼 미래 branch로 점프하지 않도록 제한하며, 전역 최근접 후보보다 0.5m 넘게 떨어진 후보만 남으면 ambiguous로 invalid 처리합니다. 이는 보수적인 연속성 휴리스틱이며 모든 U턴/역주행을 보장하지 않습니다.
- 진행 거리 = max(0, 마지막 arc - 시작 arc). 정규화는 GT_BASELINE_EP.md에 따라 GT 고정 분모 및 0~1 제한을 사용합니다. NC/DAC 마스킹과 5m 이하 일괄1 규칙은 EP에서 제거했으며 종합 PDMS 가중치는 그대로입니다. 원래 NAVSIM과 동일한 평가 프로토콜이라고 주장하지 않습니다.
- endpoint에 정확히 도달한 경우는 허용합니다. endpoint tangent 방향으로 0.05m 초과 넘어선 rollout은 endpoint clipped로 invalid입니다. 임의 직선 외삽을 하지 않습니다.
- 전체 GT 기준선이 정지일 때, rollout 중심이 기준점에서 0.25m 이내면 진행도 0; 그 이상 이동하면 측정할 기준 방향이 없어 invalid입니다. 0.25m는 수치/추종 오차 허용값이며 설정된 차선 폭과 무관합니다.
- GT가 평가 구간 후 정지하여 기준선이 끝나면, 정지점까지는 평가하지만 이를 넘어가는 rollout은 invalid일 수 있습니다. 필요시 centerline 모드를 별도 비교하세요. 두 방식의 결과를 혼합 집계하지 마세요.

## 샘플 선택

CLI 기본 `--sample-interval 0`: 시간 조건을 충족하는 모든 sample을 입력 순서대로 평가합니다. 명시적으로 1.5를 지정하면 scene별 간격 선택을 사용할 수 있습니다. 시간 부족 제외 후 간격 선택, 그 뒤 limit을 적용합니다.

`selection.json`은 간격 적용 후 목록/시각, 제외 개수, limit 후 실제 token 목록을 저장합니다. `evaluated_tokens.txt`는 실제 평가 목록입니다. Python evaluate API는 호환성을 위해 sample_interval=0 기본값을 유지합니다.

## 진단과 시각화

summary/sample result에 ep_reference 기록. sample result와 scene.json에 ep_path_info(연장 길이/종료 이유/정지 여부), diagnostics.json의 model/gt에 ep_projection(시간별 arc 위치/정책)을 기록합니다. 뷰어의 파란 EP route는 GT 차량 중심 경로이며, 점수 패널에서 모드를 확인할 수 있습니다. 기존 초록 GT reference/rollout은 rear-axle 경로라 차량 중심 오프셋만큼 차이가 나는 것이 정상입니다.

## 검증 범위

합성 차선 변경, 정지/감속 정지, 연장 경로보다 빠른 모델, 기록 끝 초과, 시간 gap, 자기 교차 branch, nuScenes 지도 route 연결 실패 우회, scene별 timestamp 샘플링, 직렬/병렬 동일 결과를 테스트합니다. 실제 사용자 token 30e55a3ec6184d8cb1944b39ba19d622의 원본 데이터는 이 환경에서 재평가하지 않았습니다.
