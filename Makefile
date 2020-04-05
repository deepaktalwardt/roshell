all:
	gcc -Wall -pedantic main.c source.c variable.c command.c history.c -o roshell
