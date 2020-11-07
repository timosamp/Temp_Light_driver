#include<stdio.h>
#include <fcntl.h>


void main(){
	char executable[] = "bytebyte";
	char *newargv[] = { executable, NULL, NULL, NULL };
	char *newenviron[] = { NULL };
	int status;

	for(int i=0; i<4; i++){
		pid_t pid = fork();
		
		if(pid == 0){
			execve(executable, newargv, newenviron);
		}
	}

	
	for(int i=0; i<4; i++){
		wait(&status);
	}

}
