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
<h4>1. 초기상태</h4>
<p align="center"><img src="https://user-images.githubusercontent.com/48746729/121833943-79184c00-cd08-11eb-9b52-c76e71d6b625.png"/></p>

<h4>2. 비례항 (P)</h4>
<p align="center"><img src="https://user-images.githubusercontent.com/48746729/121834379-9a2d6c80-cd09-11eb-873a-474651935e34.png"/></p>
<p align="center"><img src="https://user-images.githubusercontent.com/48746729/121834035-b7157000-cd08-11eb-887d-6c5513558dbd.png"/></p>

- 그래프에서 보면 Kp값이 5까지 증가하면서 SetPoint는 빨리 도달하였다. (피크점) 
- 하지만 전체적으로 오차가 너무 심하고 시스템이 불안정 한 것을 확일 할 수 있다.

<h4>3. 적분항 (P >> I)</h4>
<p align="center"><img src="https://user-images.githubusercontent.com/48746729/121834405-b204f080-cd09-11eb-9593-198c1de62f16.png"/></p>
<p align="center"><img src="https://user-images.githubusercontent.com/48746729/121834056-c72d4f80-cd08-11eb-8f63-ba9040e10676.png"/></p>

- <b>비례방식으로 드론을 제어하기엔 무리임을 확인하였다.</b> 
- <b>적분항은 시간에 걸친 오차의 누적된 합을 말한다.</b> 오차가 P항에서처럼 클 경우 적분항에 의해 시간이 지나면서 오차의 합이 쌓여 출력은 빠르게 피드백을 함으로 오차를 제거한다.
- 위 그래프를 보면 비례항 대비 오차가 줄어들었고 시스템이 나름 안정화 된 것을 볼 수 있다.

<h4>4. 미분항 (P >> I >> D)</h4>
<p align="center"><img src="https://user-images.githubusercontent.com/48746729/121834433-c0eba300-cd09-11eb-97b0-51f59ebbf13f.png"/></p>
<p align="center"><img src="https://user-images.githubusercontent.com/48746729/121834082-d3b1a800-cd08-11eb-9ab2-5f6a52307599.png"/></p>

- 미분항은 오차의 변화율을 계산하고 그 결과를 출력에 더한다. 
- 오차의 변화가 크지 않다면 미분항의 값은 작아지게 되고 출력에는 큰 영향을 미치지 않는다. (가끔 그래서 제어할 때 D를 생략하는 이중 PID가 있다.)
- 하지만 오차가 크게 변한다면 시스템의 진동을 피하기 위해서 미분항의 값은 커진다.
- 위 그래프는 적분항에 있던 오차를 변화율에 따라 제거함으로 이제 겨우 제어 할 만한 수준까지 안정화 된다.
 
## :tada: 표준 PID 알고리즘

- 상대방이 커밋한 파일은 명령어를 통해서 직접 업데이트를 하셔야 동기화가 됩니다.
- 이때 사용하는 명령어는 `git pull`과 `git fetch`가 있습니다.

```C
# master 브랜치를 pull하여 업데이트
$ git pull origin master
  
# master 브랜치를 fetch하여 업데이트
$ git fetch origin master
```

- `pull` 과 `fetch` 의 차이점은 `merge` 작업을 하느냐 안하느냐로 나뉘어지며.
- `pull` 은 `fetch` + `merge` 작업이라고 생각하시면 됩니다.


## :mag: 라이센스

![cc license](http://i.creativecommons.org/l/by/4.0/88x31.png)

이 가이드는 Creative Commons Attribution 4.0 (CCL 4.0)을 따릅니다.

