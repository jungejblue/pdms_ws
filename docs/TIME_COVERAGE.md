# 전체 sample 평가와 시간 부족 제외

기본 `pdms evaluate --out name`은 prediction PKL의 모든 sample을 대상으로 합니다.
현재 시점부터 raw GT 3.0초, raw 객체 관측 프레임 3.9초가 남아 있지 않으면
MPC 실행 전에 제외합니다. scene 전체가 짧으면 전체 sample이 제외되고,
긴 scene의 끝부분만 부족하면 해당 sample만 제외됩니다.
nuScenes는 LIDAR_TOP keyframe 시각과 scene의 lidar/annotation frame 끝 시각을,
ETRI는 infos 시각과 ego_pose/object의 ego 행 끝 시각을 사용합니다.

- excluded_samples.json: 제외 token, scenario, 남은 GT/객체 시간, 이유.
- selection.json: 시간 제외 내역과 실제 평가 token 목록.
- evaluated_tokens.txt: 이번 평가 대상으로 선택된 token.
- summary.json: input_samples, time_eligible, time_excluded, requested, valid, invalid.
- 제외 sample은 점수 및 시나리오 평균, viewer에 포함하지 않습니다.
- complete는 선택된 평가 대상에 대한 완료 여부입니다. scene 전체 coverage가 아닙니다.
- 전부 제외된 scene은 scenario_scores.json에 없으며 excluded_samples.json에서 확인합니다.
- 모든 sample이 제외되면 제외 내역을 저장하고 오류 종료합니다. 빈 뷰어는 만들지 않습니다.

순서: 시간 부족 제외 → 선택적 sample interval → limit → 평가.
limit 10은 시간 조건을 통과한 후보 중 10개이며, MPC/지도 오류가 생겨도
다른 sample로 대체하지 않습니다. tokens 목록도 시간 조건을 우회하지 않습니다.
누락 파일, 잘못된 좌표, 내부 timestamp gap, MPC 실패 및 EP 끝점 문제는
자동 제외로 숨기지 않고 기존 평가 오류로 남습니다. 이 사전 검사는 recording 끝
시각 기준이며, 모든 데이터 정합성을 보장하지 않습니다.

```bash
source scripts/setup_pdms_docker_env.sh nuscenes v1.0-trainval
pdms evaluate --out nuscenes_all_check --limit 10 --workers 1
pdms serve --run nuscenes_all_check
# 전체 평가 (worker 최대 4)
pdms evaluate --out nuscenes_all --workers 4
pdms serve --run nuscenes_all
# 간격 선택은 필요한 경우에만 명시
pdms evaluate --out nuscenes_sparse --sample-interval 1.5 --workers 4
```

TTC, EP, NC, DAC, Comfort 계산식과 고정 3초 horizon은 변경하지 않습니다.
기존 runs 결과도 변경하지 않습니다. 새 out 경로로 실행하세요.
