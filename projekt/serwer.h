#ifndef SERWER_H
#define SERWER_H

#include <QDialog>

namespace Ui {
class serwer;
}

class serwer : public QDialog
{
    Q_OBJECT

public:
    explicit serwer(QWidget *parent = nullptr);
    ~serwer();
    QString getIP() const;
    quint16 getPort() const;
    QString getTryb() const;

private:
    Ui::serwer *ui;
    QString  ip="";
    QString  port="";
};

#endif // SERWER_H
