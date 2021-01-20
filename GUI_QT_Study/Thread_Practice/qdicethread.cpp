#include "qdicethread.h"
#include <Qtime>

QDiceThread::QDiceThread()
{

}

void QDiceThread::diceBegin(){
    m_paused = false;
}

void QDiceThread::dicePause(){
    m_paused = true;
}

void QDiceThread::stopThread(){
    m_stop = true;
}

void QDiceThread::run(){
    m_stop = false;
    m_seq = 0;
    qsrand(QTime::currentTime().msec());
    while(!m_stop){
        if(!m_paused){
            m_diceValue = qrand();
            m_diceValue = (m_diceValue % 6) + 1;
            m_seq++;
            emit newValue(m_seq, m_diceValue);
        }
        msleep(500);
    }
    quit();
}
