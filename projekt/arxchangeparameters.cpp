#include "arxchangeparameters.h"
#include <QDebug>
#include <QMessageBox>
#include <QStringList>
#include "ui_arxchangeparameters.h"
ArxChangeParameters::ArxChangeParameters(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::ArxChangeParameters)
    , simulation(&Simulation::get_instance())
{
    ui->setupUi(this);

    ui->arx_noise_input->setValue(simulation->arx->get_noise());
    ui->arx_noisetype_input->setCurrentIndex(static_cast<int>(simulation->arx->get_noise_type()));
    ui->arx_delay_input->setValue(simulation->arx->get_delay());

    QStringList a_values;
    for (double value : simulation->arx->get_a()) {
        a_values.push_back(QString::number(value));
    }
    ui->arx_a_input->setText(a_values.join(","));

    QStringList b_values;
    for (double value : simulation->arx->get_b()) {
        b_values.push_back(QString::number(value));
    }
    ui->arx_b_input->setText(b_values.join(","));
    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &QWidget::close);
    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &ArxChangeParameters::onOkClicked);
}

ArxChangeParameters::~ArxChangeParameters()
{
    delete ui;
}
void ArxChangeParameters::onOkClicked()
{
    // noise
    simulation->arx->set_noise(ui->arx_noise_input->value());

    // noise type
    simulation->arx->set_noise_type(static_cast<NoiseType>(ui->arx_noisetype_input->currentIndex()));

    // delay
    simulation->arx->set_delay(ui->arx_delay_input->value());

    // a coefficients
    QStringList a_values = ui->arx_a_input->text().split(",");
    std::vector<float> a;
    for (const QString &value : a_values) {
        if (!value.isEmpty()) {
            bool ok = false;
            float val = value.toFloat(&ok);
            if (ok)
                a.push_back(val);
        }
    }
    simulation->arx->set_a(a);

    // b coefficients
    QStringList b_values = ui->arx_b_input->text().split(",");
    std::vector<float> b;
    for (const QString &value : b_values) {
        if (!value.isEmpty()) {
            bool ok = false;
            float val = value.toFloat(&ok);
            if (ok)
                b.push_back(val);
        }
    }
    simulation->arx->set_b(b);

    // zamknij okno
    this->accept(); // lub close() jeśli nie chcesz exec()
}
