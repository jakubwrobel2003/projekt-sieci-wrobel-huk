#include "serwer.h"
#include "ui_serwer.h"

serwer::serwer(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::serwer)
{
    ui->setupUi(this);


    connect(ui->klient, &QCheckBox::toggled, this, [=](bool checked) {
        if (checked)
            ui->serwer_2->setChecked(false);

    });

    connect(ui->serwer_2, &QCheckBox::toggled, this, [=](bool checked) {
        if (checked) ui->klient->setChecked(false);
    });

}

serwer::~serwer()
{
    delete ui;
}
QString serwer::getIP() const {
    return ui->IP->toPlainText(); // jeśli QTextEdit
}

quint16 serwer::getPort() const {
    return ui->PORT->toPlainText().toUShort();
}

QString serwer::getTryb() const {
    if (ui->klient->isChecked()) return "klient";
    else if (ui->serwer_2->isChecked()) return "serwer";
    return "brak";
}


