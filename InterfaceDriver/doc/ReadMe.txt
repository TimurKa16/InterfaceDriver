
????????? ??? ?????????? ?????? ??????????

main():
    ?????????????
    ?????? ?????
    ?????? ??????? ?? COM-?????
    ?????????? ?????????
    ?????? ??????? ???????????
    

??????????

CpuTimerIsr():
    ?????????????? ????????
    1 ??????? = 1000 ?????

SciRxIsr():
    ????? ? ????????? ?????, ???????????? ?? COM-?????

adca1_isr():
	????????? ???

???????? ????????????:
	???????? ??????????? ?? SCI ? ?????? firmwareBuffer ? ????????? ??????? RAM
	?? ???????? ????? ????????, ??????? ? ????????? (?????? ??????)
	? ?????????? ??????????? ??????? ????? ???????? ? ????????? ??????? RAM
	???? ???? ????????, ?? ?????????
	???? ??? ????????, ?? ??????? ? ???????? ????????
	
??????????:

	GPIO51 - ??????? Renishaw, ?????????? ?? ????????? ?? ???????
	GPIO52 - ???????? ??????? ?????????
	GPIO53 - ??????? ??????? ???????? ????????
	GPIO54 - ??????? ??????? ??????? ????????
	GPIO55 - ?????????????? ??????? ??????? ?????????

	?? ???????? ? ??????????? ??????????? ?? ????????? ???????? ?????????? 48 ?,
	??????? ????? ?? ???????? (GPIO54) ???????????? ? (GPIO51, GPIO55)
	????? ???? ????? "?? ????????"!!!
	
	