# Drone PID Study
<h1 align="center">PID 알고리즘 이해하기</h1>

<p align="center"><img src="https://user-images.githubusercontent.com/48746729/121832266-a6fb9180-cd04-11eb-8e19-e09455ad33fb.png" width="720" /></p>

<p align="center">PID 블록 다이어그램</p>

## :pencil2: 설명

- PID는 비례(Proportion), 적분(Integral), 미분(Differential) 를 통한 제어 기법이다.
- 기본적으로 피드백 구조로 되어 있다.


### :pencil2: 특징
- 제어하고자 하는 대상의 입력 값(INPUT)을 측정하여 이를 목표 설정값(SetPoint)과 비교하여 오차를 계산한다.
- 이 때 오차값을 이용하여 제어에 필요한 제어값(OUTPUT)을 계산하고, 이 제어값은 다시 피드백 되어 제어하고자 하는 대상의 입력으로 사용되는 구조다.

쉽게 날려서 설명하자면 입력 >> 목표치 도달설정 >> 오차값으로 필요 제어량 계산 >> 피드백 이다.


### :pencil2: 수식 이해하기
<p align="center"><img src="https://user-images.githubusercontent.com/48746729/121833193-c7c4e680-cd06-11eb-9dfa-8901c80c30a6.png" width="480" /></p>
<p align="center"><img src="https://user-images.githubusercontent.com/48746729/121833277-f478fe00-cd06-11eb-9636-39aa591a04ed.png" width="320" /></p>

<b>각각의 역할은 다음과 같다.</b>

- 비례항 : 현제 상태에서의 오차 값의 크기에 비례한 제어를 한다.
- 적분항 : 일정한 상태로 유지되는 오차를 없애는 작용한 한다.
- 미분항 : 출력값의 급격한 변화에 제동을 걸어 목표값을 지나가 버리는 오버슛을 줄여 안정성을 향상시킨다.

<h3>비례, 적분, 미분 제어에 따른 변화</h3>
<b>1. 초기상태</b>
<p align="center"><img src="https://user-images.githubusercontent.com/48746729/121833943-79184c00-cd08-11eb-9b52-c76e71d6b625.png"/></p>

<b>2. 비례항 (P)</b>
<p align="center"><img src="https://user-images.githubusercontent.com/48746729/121834379-9a2d6c80-cd09-11eb-873a-474651935e34.png"/></p>
<p align="center"><img src="https://user-images.githubusercontent.com/48746729/121834035-b7157000-cd08-11eb-887d-6c5513558dbd.png"/></p>

3. 적분항 (I)
<p align="center"><img src="https://user-images.githubusercontent.com/48746729/121834405-b204f080-cd09-11eb-9593-198c1de62f16.png"/></p>
<p align="center"><img src="https://user-images.githubusercontent.com/48746729/121834056-c72d4f80-cd08-11eb-8f63-ba9040e10676.png"/></p>

4. 미분항 (D)
<p align="center"><img src="https://user-images.githubusercontent.com/48746729/121834433-c0eba300-cd09-11eb-97b0-51f59ebbf13f.png"/></p>
<p align="center"><img src="https://user-images.githubusercontent.com/48746729/121834082-d3b1a800-cd08-11eb-9ab2-5f6a52307599.png"/></p>

 ![Git add files](https://www.pigno.se/static/assets/images/git_tutorial_refer_add.png)

- Staged 상태의 파일은 아직 기록된 상태가 아닙니다.
- 파일의 기록을 위해서는 `커밋` 작업이 필요합니다.
- `git commit` 명령을 통해 Staged 상태의 파일을 커밋할 수 있습니다.

![Git commit](https://www.pigno.se/static/assets/images/git_tutorial_refer_commit.png)

- `-m` 옵션을 이용하여 커밋 메시지를 작성하는 것을 권장합니다.
- 실수로 커밋을 하여, 다시 커밋을 할 경우 커밋을 덮어씌울 수 있습니다. 이때 `--amend` 옵션을 이용합니다.

```bash
$ git add *
$ git commit -m "UI 레이아웃 이슈 수정."

# 수정사항 발생
$ git add *
$ git commit -m "UI 레이아웃 이슈 수정 및 관리자 벨리데이션 추가." --amend
```

## :tada: 소스 업데이트

- 상대방이 커밋한 파일은 명령어를 통해서 직접 업데이트를 하셔야 동기화가 됩니다.
- 이때 사용하는 명령어는 `git pull`과 `git fetch`가 있습니다.

```bash
# master 브랜치를 pull하여 업데이트
$ git pull origin master
  
# master 브랜치를 fetch하여 업데이트
$ git fetch origin master
```

- `pull` 과 `fetch` 의 차이점은 `merge` 작업을 하느냐 안하느냐로 나뉘어지며.
- `pull` 은 `fetch` + `merge` 작업이라고 생각하시면 됩니다.

## :clock11: 소스 복원

- 여러분이 git을 쓰는 이유중에 중요한 부분을 차지하는 영역입니다.
- 정상적으로 커밋된 히스토리는, 리비전으로 git에 관리됩니다.
- 실수로 잘못 작업하였거나, 예전 버전으로 롤백하여 적용할 경우 여러분은 예전 버전으로 리셋하실 수 있습니다.
- 리셋은 `git reset` 명령을 사용합니다.

```bash
$ git reset HEAD^ --soft
```

- `git reset` 다음 인수로는 되돌리는 버전의 위치를 가리킵니다.
- 현재위치(HEAD)를 기준하여 상대적인 위치를 설정하거나, 특정 버전 리비전 고유의 해시값을 지정합니다.
- HEAD를 확인하시고 싶으면 `git reflog` 명령을 이용합니다.

- `git reset`의 옵션 중 리셋 특성을 정하는 `--soft, --hard, --mixed` 옵션이 있습니다.
- 위 옵션은 아래에서 자세히 설명합니다.

 - `--soft`는 약한특성의 리셋입니다, 되돌릴 때 기존의 인덱스와 워킹트리를 보존합니다.
 - `--hard`는 강한특성의 리셋입니다, 되돌릴 때 기존의 인덱스와 워킹트리를 버립니다.
 - `--mixed`는 중간특성의 리셋입니다, 되돌릴 때 기존의 인덱스는 버리고 워킹트리를 보존합니다.

- 되돌리는 위치의 경우 아래와 같은 타입이 있습니다.

```bash
# 바로 이전 단계로 인덱스와 워킹트리를 버리고 리셋.
$ git reset HEAD^ --hard 
    
# 바로 두번째 전 단계로 인덱스와 워킹트리를 버리고 리셋.
$ git reset HEAD~2 --hard 
    
# 특정 리비전의 기록으로 인덱스는 버리고 워킹트리를 보존하여 리셋.
$ git reset 991ee8c --mixed
```

## :seedling: 브랜치

- 브랜치는 한국말로 가지(branch)입니다.
- git에서는 마치 가지를 펼치듯 하나의 근본에서 여러 갈래로 쪼개어 관리할 수 있습니다.

 ![Git branch](https://www.pigno.se/static/assets/images/git_tutorial_branch.png)
 이미지 출처 [StackOverflow](http://stackoverflow.com/questions/23142731/push-a-feature-branch-to-develop-branch-using-git)


- branch의 특징은 아래와 같습니다.

 - 기본은 master 브랜치라고 불리며, 필수로 제공되는 브랜치이다.
 - 서로다른 브랜치들은 같은 조상을 가지고 있다.

- 브랜치를 새로 만드신다면 `git branch [브랜치명]`으로 생성합니다.
- 아래 명령라인에서는 new라는 브랜치를 생성하고 있습니다.

```bash
$ git branch new
```

- master 기준으로 new를 브랜치(가지치기)하면 master와 똑같은 소스코드가 new에도 적용됩니다.
- 하지만 이 이후로 new에서 코드를 수정하면, master와 new는 서로 다른 코드가 되기 때문에 갈라집니다.

- 생성된 new 브랜치로 접속하기 위해서는 `git checkout [브랜치명]`을 이용합니다.

```bash
$ git checkout new
```

 - 생성과정과 브랜치 이동과정을 동시에 하고자 하면 `git checkout`에 `-b` 옵션을 이용합니다.

```bash
# 브랜치 new를 생성과 동시에 체크아웃.
$ git checkout -b new
```

- 생성한 브랜치는 현재 로컬에 저장되어 있습니다.
- 협업 작업에서는 생성한 브랜치를 원격 저장소에 등록해주어야 합니다.
- 이때는 `git push [브랜치명]`을 이용합니다.

```bash
$ git push new
```

- 브랜치 생성 및 등록의 과정은 아래 화면과 같습니다.

 ![Git new branch](https://www.pigno.se/static/assets/images/git_tutorial_new_branch.png)

- 브랜치의 삭제는 `git branch` 명령에서 `-d` 옵션을 사용합니다.

 ![Git delete branch](https://www.pigno.se/static/assets/images/git_tutorial_delete_branch.png)

- 삭제된 브랜치 또한 원격 저장소에 반영을 해야합니다.
- 이때 브랜치 명 앞에 콜론(:)을 붙여주어야 하니 이 점 주의해주세요.

## :fearful: 소스 병합

- 브랜치를 사용하는 과정에서 가장 중요한 머지와 리베이스 등의 병합 기법입니다.
- 서로 다른 브랜치에서 서로 다른 코드가 개발되었고, 실제 배포에서 이를 합쳐야 할 때 사용합니다.
- 병합 방식에서는 크게 `git merge`와 `git rebase`가 존재합니다.
- 머지 방식과 리베이스 방식의 차이는 아래 이미지를 확인해주세요.

 ![Difference between merge and rebase](https://www.pigno.se/static/assets/images/git_tutorial_merge_rebase.png)
 
 이미지 출처 [http://git.mikeward.org/](http://git.mikeward.org/)

- 아래는 머지해야 하는 상황을 구현해봤습니다.
- `master`에서 `sub` branch가 생성되었으며, master 브랜치에서 sub 브랜치를 머지하고자 합니다.
- 파일 구성은 아래와 같습니다.

```plaintext
* master -> some_file.txt의 내용
* 1번째 단계 HEAD
I'm a file.
```

```plaintext
* sub -> some_file.txt의 내용
* 2번째 단계 HEAD (최신)
I'm a file.
    
Inserted new line from the sub branch.
```

```bash
$ git checkout -f master
$ git merge sub
# 현재 브랜치 master, 대상 브랜치 sub.
# master에서 sub를 머지합니다.
# HEAD -> master
# sub  -> sub
```

- 머지 이후 master에서 파일을 보면, 아래와 같은 내용을 얻습니다.

```plaintext
* merge 이후 master -> some_file.txt
I'm a file.
    
Inserted new line from the sub branch.
```

## :sob: 충돌과 해결

- git으로 merge, rebase 수행시 충돌(conflict)가 발생 할 수 있습니다.
- 이는 같은 조상을 기준으로, 서로 다른 두개의 브랜치가 같은 소스코드를 변경했을 때 발생합니다.

```plaintext
* master -> some_file.txt의 내용
Apple
```

- 위는 `master` 브랜치의 some_file.txt의 내용이다.
- 아래는 해당 브랜치를 복제한 `sub` 브랜치이며, 복제 이후 한번 내용을 수정하였습니다.

```plaintext
* sub -> some_file.txt의 내용
* 2번째 단계 HEAD
Banana
```

- 이후 master에서도 내용을 변경하여 버전을 업데이트 합니다.
 
```plaintext
* master -> some_file.txt의 내용
* 2번째 단계 HEAD(sub랑 단계가 겹침)
Strawberry
```

- 둘 모두 버전이 같으나 같은 라인에서 변경사항이 발생했습니다.
- 이 경우 충돌이 발생합니다.
- 충돌이 발생한 some_file.txt를 열어보면 아래와 같은 내용을 보실 수 있습니다.

```plaintext
* 머지 이후 master -> some_file.txt (충돌)
<<<<<<< HEAD
Strawberry
=======
Banana
>>>>>>> sub
```

- 여기서 `HEAD`는 현재 브랜치(master)를 의미합니다.
- HEAD와 sub의 각각 내용을 보여주고 있는데 꺽쇠(<, >), 이퀄(=)기호가 없도록 문장 하나를 선택해서 반영해주어야
- 충돌이 해결 될 수 있습니다.
- 여기서는 `master` 브랜치의 Strawberry를 선택하여 충돌을 해결하겠습니다.

```plaintext
* 머지 이후 master -> some_file.txt (수정)
Strawberry
```

- 수정이 되었다면 머지 해결을 위해 `git add`와 `git commit`으로 충돌(conflict)을 해결하세요.

```bash
$ git add *
$ git commit -m "Solved the conflict issue."
```

## :mag: 라이센스

![cc license](http://i.creativecommons.org/l/by/4.0/88x31.png)

이 가이드는 Creative Commons Attribution 4.0 (CCL 4.0)을 따릅니다.

