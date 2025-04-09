#ifndef ARXCHANGEPARAMETERS_H
#define ARXCHANGEPARAMETERS_H

#include <QDialog>
#include "simulation.h"

namespace Ui {
class ArxChangeParameters;
}

class ArxChangeParameters : public QDialog
{
    Q_OBJECT

public:
    explicit ArxChangeParameters(QWidget *parent = nullptr);
    ~ArxChangeParameters();
    void on_arx_noise_input_editingFinished();
    void on_arx_noisetype_input_currentIndexChanged(int index);
    void on_arx_delay_input_editingFinished();
    void on_arx_b_input_editingFinished();
    void on_arx_a_input_editingFinished();

private:
    Ui::ArxChangeParameters *ui;
    void onOkClicked();

    Simulation *simulation;
};

#endif // ARXCHANGEPARAMETERS_H
