#include <stdio.h>

int press[6][8];
int puzzle[6][8];

bool guess(){
    int i, j;
    for(i = 2; i <= 5; i++){
        for(j = 1; j <= 6; j++){
            press[i][j] = (press[i-1][j] + puzzle[i-1][j]
                           + press[i-1][j-1] + press[i-2][j] + press[i-1][j+1]) % 2;
        }
    }

    for( j = 1; j <= 6; j++){
        if(puzzle[5][j] != (press[5][j] + press[5][j-1] + press[5][j+1] + press[4][j]) % 2)
            return false;
    }
    return true;
}

void process(){
    int c;
    for(c = 1; c <=6; c++)
        press[1][c] = 0;

    while(!guess()){
        press[1][1]++;
        c = 1;
        while(press[1][c] > 1){
            press[1][c] = 0;
            c++;
            press[1][c]++;
        }
    }
}

int main()
{
    int t, i, n, j;
    // 初始化0行的所有元素
    for(i = 0; i < 8; i++) press[0][i] = puzzle[0][i] = 0;
    // 初始化0列,7列的所有元素
    for(i = 1; i < 6; i++) press[i][0] = puzzle[i][0] = press[i][7] = puzzle[i][7] = 0;

    // 注意从第二行开始有数据，所以这里需要调整初始位置
    for(i = 1; i <=5; i++)
        for(j = 1; j <= 6; j++)
            scanf("%d", &puzzle[i][j]);

    process();

    for(i = 1; i <=5; ++i){
        for(j = 1; j <=6; j++){
            printf("%d ", press[i][j]);
        }
        printf("\n");
    }
    return 0;
}