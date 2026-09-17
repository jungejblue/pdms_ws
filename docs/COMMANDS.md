# 추가 명령

모든 명령은 저장소 루트에서 `source pdms/bin/activate` 후 실행합니다.

## 환경 확인

```bash
python --version
python -m pip check
pdms --help
```

## SSH 터널

서버:

```bash
pdms serve --run runs/vad_full --host 127.0.0.1 --port 7200
```

로컬 컴퓨터의 별도 터미널:

```bash
ssh -N -L 7200:127.0.0.1:7200 USER@SERVER
```

로컬 브라우저: http://localhost:7200

## 선택적인 정적 HTML / PNG

localhost 뷰어에는 HTML 생성이 필요하지 않습니다. 정적 파일도 필요할 때만 사용합니다.

```bash
pdms evaluate \
  --raw-pkl /path/to/etri_infos_temporal_val.pkl \
  --data-root /path/to/ETRI/val \
  --planning-pkl /path/to/planning.pkl \
  --out runs/vad_with_html \
  --visualize 20
```

점수가 낮은 valid 샘플 20개의 HTML/PNG를 만듭니다. 이미 계산한 한 샘플은 다음으로 생성합니다.

```bash
pdms visualize --sample-dir runs/vad_full/samples/ARTIFACT_ID
```

## 입력 검사 / 평가 옵션

```bash
pdms inspect --help
pdms evaluate --help
pdms serve --help
```

기존 `etri-pdms` 명령과 `--pred` / `--infos` 이름도 호환 alias로 유지합니다. 새 사용자는 README의 `pdms`, `--planning-pkl`, `--raw-pkl`을 사용하면 됩니다.
