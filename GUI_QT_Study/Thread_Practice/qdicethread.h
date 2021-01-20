#ifndef QDICETHREAD_H
#define QDICETHREAD_H

#include <QThread>

class QDiceThread : public QThread{
    Q_OBJECT
private:
    int m_seq = 0;
    int m_diceValue = 0;
    bool m_paused = true;
    bool m_stop = false;

protected:
    void run() override;
public:
    QDiceThread();
    void diceBegin();
    void dicePause();
    void stopThread();

signals:
    void newValue(int seq, int diceValue);
};

#endif // QDICETHREAD_H
