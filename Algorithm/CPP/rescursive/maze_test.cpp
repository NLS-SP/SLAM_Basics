//
// Created by Robotics_qi on 2020/3/27.
//

#include <stdio.h>
#include <memory.h>
#define MAXIN 75
char board[MAXIN+2][MAXIN+2];

int minstep, w, h, to[4][2] = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
char dir[4][6] = {"down", "right", "up", "left"};
bool mark[MAXIN+2][MAXIN+2];        // 定义标记数组

void Search(int now_x, int now_y, int end_x, int end_y, int step, int f){
    if(step > minstep) return; // 当前路径数大于minstep时，返回优化策略
    if(now_x == end_x && now_y == end_y){ // 到达终点
        if(minstep > step) minstep = step;
        return;
    }
    for(int i = 0; i < 4; i++){
        int x = now_x + to[i][0];
        int y = now_y + to[i][1];
        if((x > -1) && (x < w+2) && (y > -1) && (y < h+1) && (((board[y][x] == ' ') && (mark[y][x] == false)) || (mark[y][x] == false)
            || (x == end_x) && (y == end_y) && (board[y][x] == 'X'))){
            printf("go: %s\n", dir[i]);
            mark[y][x] = true;
            if(f == i) Search(x, y, end_x, end_y, step, i);
            else Search(x, y, end_x, end_y, step+1, i);
            mark[y][x] = false;
        }
    }
    if(f != -1)
        printf("end step = %d, %s, (%d %d), (%d, %d)\n", step, dir[f], now_x, now_y, end_x, end_y);
}

int main()
{
    int board_num = 0;      // 记录第几号棋盘
    while(scanf("%d %d", &w, &h)){
        if(w == 0 && h == 0) break;         // 遇到0 0 的输入就退出循环
        board_num++;
        printf("Board#%d:\n", board_num);
        int i, j;
        // 初始化第0行和第0列的所有元素
        for(int i = 0; i < MAXIN+2; i++) board[0][i] = board[i][0] = ' ';
        for(i = 1; i <= h; i++){    // 读入矩形板的布局
            getchar();
            for(j = 1; j <= w; j++) board[i][j] = getchar();
        }
        // 将最下边和最右边两条边初始化为空，之前初始化的格子跳过
        for(i = 0; i <= w; i++) board[h+1][i+1] = ' ';
        for(i = 0; i <= h; i++) board[i+1][w+1] = ' ';
        int begin_x, begin_y, end_x, end_y, count = 0;
        while(scanf("%d %d %d %d") && begin_x > 0){
            count++;
            minstep = 10000;
            memset(mark, false, sizeof(mark));
            // 递归搜索
            Search(begin_x, begin_y, end_x, end_y, 0, -1);
            // 输出结果
            if(minstep < 10000) printf("Pair %d: %d segments.\n", count, minstep);
            else printf("Pair %d: impossible.\n", count);
        }
        printf("\n");
    }
    return 0;
}