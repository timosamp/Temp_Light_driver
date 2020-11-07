#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>


void main(){
	
	int i=0, j=0, fd, ret;
	char buf[64], path[64], c;
	pid_t pid = getpid();

	sprintf(path, "/dev/lunix1-batt");

	fd = open(path, O_RDONLY);
        if (fd < 0) {
                printf("Can't open file\n");
                exit(1);
        }
	printf("Open success\n");

	while(j < 4)
	{
		while(c != '\n'){
			//ret = read(fd, buf + i, 1);
			ret = read(fd, &c, 1);
	
			if(ret == 1){
				//printf("return 1 byte: %c\n", buf[i]);
				if(c != '\n')
				{
					printf("mypid: %ld\n", pid);
					printf("return 1 byte: %c\n", c);
				}else{
					printf("\n");
				}
			}else{
				printf("read didn't return 1 byte\n");
			}
		}

		c = '0';
		j++;
	}

	if(close(fd) == 0)
	{
		printf("close succesfully\n");
	}else{
		printf("close failed\n");
	}
	
}
