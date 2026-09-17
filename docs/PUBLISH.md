# GitHub에 pdms_ws 게시

이 소스 패키지는 원격 저장소 생성이나 업로드를 실행하지 않습니다. GitHub에서 본인 계정에 빈 `pdms_ws` 저장소를 만든 뒤 저장소 루트에서 실행하세요.

```bash
git init -b main
git add README.md pyproject.toml LICENSE NOTICE.md .gitignore .github src configs docs tests licenses scripts
git status --short
git diff --cached --stat
```

추가되는 파일은 소스·설정·문서·테스트뿐인지 확인하세요. 원본 데이터, planning PKL, 학습 weight, 평가 결과, 가상환경은 포함하지 않습니다. `.gitignore`가 일반적인 데이터 파일과 `pdms/`, `data/`, `runs/` 등을 제외합니다.

```bash
git commit -m "Add PDMS evaluation and localhost viewer"

# 본인의 GitHub ID로 변경
export PDMS_GITHUB_OWNER="YOUR_GITHUB_ID"
git remote add origin "https://github.com/${PDMS_GITHUB_OWNER}/pdms_ws.git"
git push -u origin main
```

외부 사용자는 게시 후 다음처럼 받습니다.

```bash
export PDMS_GITHUB_OWNER="REPOSITORY_OWNER"
git clone "https://github.com/${PDMS_GITHUB_OWNER}/pdms_ws.git"
cd pdms_ws
python3 -m venv pdms
source pdms/bin/activate
python -m pip install -e '.[viewer]'
python -m pip install torch --index-url https://download.pytorch.org/whl/cpu
```

라이선스는 Apache-2.0이며 포함된 NAVSIM 파생 코드의 출처/라이선스를 NOTICE와 licenses에 보존했습니다. ETRI 등 데이터셋의 재배포 권한은 코드 라이선스와 별개입니다.
