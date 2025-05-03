#include "mainwindow.h"
#include <QFile>
#include <QFileDialog>
#include <QTimer>
#include "./ui_mainwindow.h"
#include "arxchangeparameters.h"
#include "exportdialog.h"
#include <QThread>
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , simulation(Simulation::get_instance())
{
    ui->setupUi(this);

    init();
    this->simulation.pid->set_integral_mode_pullout(!this->ui->radioButton->isChecked());
    connect(this->ui->radioButton,
            &QRadioButton::toggled,
            this,
            &MainWindow::on_radioButton_toggled);
    connect(&this->simulation, &Simulation::simulation_start, this, &MainWindow::simulation_start);
    connect(&this->simulation, &Simulation::simulation_stop, this, &MainWindow::simulation_stop);

    connect(this->ui->action_save_as, &QAction::triggered, this, &MainWindow::action_save_as);
    connect(this->ui->action_open, &QAction::triggered, this, &MainWindow::action_open);
    connect(this->ui->action_export,
            &QAction::triggered,
            this,
            &MainWindow::action_simulation_export);
    connect(this->ui->action_simulation_open,
            &QAction::triggered,
            this,
            &MainWindow::action_simulation_open);

    connect(ui->ArxButton, &QPushButton::clicked, this, &MainWindow::openArxDialog);

    connect(&simulation, &Simulation::config_server_received,
            this, &MainWindow::apply_config);

}




void MainWindow::action_simulation_open()
{
    qDebug() << "open";

    QString file_name = QFileDialog::getOpenFileName(this,
                                                     "Open simulated csv",
                                                     "",
                                                     "Simulated data (*.csv)");

    QFile file(file_name);

    QString header;

    if (file.open(QIODevice::ReadOnly)) {
        header = file.readLine();

        QStringList header_parts = header.split(",");
        if (header_parts.size() != 9) {
            for (auto &part : header_parts) {
                qDebug() << part;
            }
            qDebug() << header_parts.size();
            qDebug() << "invalid header";
            return;
        }

        this->simulation.reset();

        while (!file.atEnd()) {
            QString line = file.readLine();

            QStringList parts = line.split(",");

            // parse to frame
            SimulationFrame frame;

            frame.tick = parts[0].toInt();
            frame.i = parts[1].toFloat();
            frame.p = parts[2].toFloat();
            frame.d = parts[3].toFloat();
            frame.pid_output = parts[4].toFloat();
            frame.geneartor_output = parts[5].toFloat();
            frame.error = parts[6].toFloat();
            frame.arx_output = parts[7].toFloat();
            frame.noise = parts[8].toFloat();

            emit this->simulation.add_series("I", frame.i, ChartPosition::top);
            emit this->simulation.add_series("P", frame.p, ChartPosition::top);
            emit this->simulation.add_series("D", frame.d, ChartPosition::top);
            emit this->simulation.add_series("PID Output", frame.pid_output, ChartPosition::top);

            emit this->simulation.add_series("Generator Output",
                                             frame.geneartor_output,
                                             ChartPosition::middle);
            emit this->simulation.add_series("Error", frame.error, ChartPosition::middle);

            emit this->simulation.add_series("ARX Output", frame.arx_output, ChartPosition::bottom);
            emit this->simulation.add_series("Noise", frame.noise, ChartPosition::middle);

            this->simulation.increment_tick();
            this->simulation.frames.push_back(frame);
        }

        file.close();
    }
}



void MainWindow::action_simulation_export()
{
    this->simulation.stop();

    ExportDialog dialog;
    bool result = dialog.exec();

    if (!result)
        return;

    ExportChecked checked = dialog.get_checked();

    QString file_name = QFileDialog::getSaveFileName(this,
                                                     "Export simulation",
                                                     "",
                                                     "CSV files (*.csv)");

    QFile file(file_name);

    const QString SEPARATOR = ",";

    QString header = "Time" + SEPARATOR;

    qDebug() << (checked.error);

    if (checked.pid_i)
        header += "PID I" + SEPARATOR;
    if (checked.pid_p)
        header += "PID P" + SEPARATOR;
    if (checked.pid_d)
        header += "PID D" + SEPARATOR;
    if (checked.pid_output)
        header += "PID Output" + SEPARATOR;
    if (checked.error)
        header += "Error" + SEPARATOR;

    if (checked.generator_output)
        header += "Generator Output" + SEPARATOR;
    ;
    if (checked.arx_output)
        header += "ARX Output" + SEPARATOR;
    if (checked.arx_noise)
        header += "ARX Noise" + SEPARATOR;

    header.chop(1);

    if (file.open(QIODevice::WriteOnly)) {
        file.write(header.toUtf8());
        file.write("\n");

        for (auto &frame : this->simulation.frames) {
            QString result = QString::number(frame.tick) + SEPARATOR;
            if (checked.pid_i)
                result += QString::number(frame.i) + SEPARATOR;
            if (checked.pid_p)
                result += QString::number(frame.p) + SEPARATOR;
            if (checked.pid_d)
                result += QString::number(frame.d) + SEPARATOR;
            if (checked.pid_output)
                result += QString::number(frame.pid_output) + SEPARATOR;
            if (checked.generator_output)
                result += QString::number(frame.geneartor_output) + SEPARATOR;
            if (checked.error)
                result += QString::number(frame.error) + SEPARATOR;
            if (checked.arx_output)
                result += QString::number(frame.arx_output) + SEPARATOR;
            if (checked.arx_noise)
                result += QString::number(frame.noise) + SEPARATOR;

            result.chop(1);
            result += "\n";

            file.write(result.toUtf8());
        }

        file.close();
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::action_save_as()
{
    qDebug() << "save as";

    std::vector<std::byte> data{this->simulation.serialize()};

    QString file_name = QFileDialog::getSaveFileName(this,
                                                     "Save simulation",
                                                     "",
                                                     "Simulation files (*.dat)");

    QFile file(file_name);
    if (file.open(QIODevice::WriteOnly)) {
        file.write(reinterpret_cast<const char *>(data.data()), data.size());
        file.close();
    }

    qDebug() << "saved";
}

void MainWindow::action_open()
{
    qDebug() << "open";

    QString file_name = QFileDialog::getOpenFileName(this,
                                                     "Open simulation",
                                                     "",
                                                     "Simulation files (*.dat)");

    QFile file(file_name);
    if (file.open(QIODevice::ReadOnly)) {
        std::vector<std::byte> data(file.size());
        file.read(reinterpret_cast<char *>(data.data()), data.size());
        file.close();

        this->simulation.deserialize(data);
    }

    this->init();

    qDebug() << "opened";
}


void MainWindow::apply_config(const ConfigServerPacket& packet)
{
    ui->simulation_interval_input->setValue(packet.interval);
    ui->simulation_duration_input->setValue(packet.duration);
    ui->pid_kp_input->setValue(packet.pid_kp);
    ui->pid_ti_input->setValue(packet.pid_ti);
    ui->pid_td_input->setValue(packet.pid_td);
    ui->radioButton->setChecked(!packet.pid_ti_pullout);
    ui->generator_amplitude_input->setValue(packet.generator_amplitude);
    ui->generator_frequency_input->setValue(packet.generator_frequency);
    ui->generator_generatortype_input->setCurrentIndex(static_cast<int>(packet.generator_type));
}


void MainWindow::init()
{
    // simulation

    // this->ui->simulation_ticks_per_second_input->setValue(this->simulation.get_ticks_per_second());

    this->ui->simulation_interval_input->setValue(this->simulation.get_interval());
    this->simulation.stop();

    this->ui->simulation_duration_input->setValue(this->simulation.durration);

    // pid

    this->ui->pid_kp_input->setValue(this->simulation.pid->get_kp());
    this->ui->pid_ti_input->setValue(this->simulation.pid->get_ti());
    this->ui->pid_td_input->setValue(this->simulation.pid->get_td());

    // generator

    this->ui->generator_amplitude_input->setValue(this->simulation.generator->get_amplitude());
    this->ui->generator_frequency_input->setValue(this->simulation.generator->get_frequency());
    this->ui->generator_generatortype_input->setCurrentIndex(
        static_cast<int>(this->simulation.generator->get_type()));
    this->ui->generator_infill_input->setValue(this->simulation.generator->get_infill());
    // arx
    /*
    this->ui->arx_noise_input->setValue(this->simulation.arx->get_noise());
    this->ui->arx_noisetype_input->setCurrentIndex(
        static_cast<int>(this->simulation.arx->get_noise_type()));
    this->ui->arx_delay_input->setValue(this->simulation.arx->get_delay());

    QStringList a_values;

    for (double value : this->simulation.arx->get_a()) {
        a_values.push_back(QString::number(value));
    }

    this->ui->arx_a_input->setText(a_values.join(","));

    QStringList b_values;

    for (double value : this->simulation.arx->get_b()) {
        b_values.push_back(QString::number(value));
    }

    this->ui->arx_b_input->setText(b_values.join(","));
*/

}

void MainWindow::simulation_start()
{
    this->ui->simulation_start_button->setEnabled(false);
    this->ui->simulation_stop_button->setEnabled(true);
}

void MainWindow::simulation_stop()
{
    this->ui->simulation_start_button->setEnabled(true);
    this->ui->simulation_stop_button->setEnabled(false);
}

void MainWindow::on_simulation_start_button_clicked()
{
    if (this->simulation.durration == 0) {
        this->simulation.start();
    } else {
        this->simulation.start();

        auto timer = new QTimer(this);
        timer->setSingleShot(true);
        connect(timer, &QTimer::timeout, [this]() { this->simulation.stop(); });

        timer->start(this->simulation.durration * 1000);
    }
}

void MainWindow::on_simulation_stop_button_clicked()
{
    this->simulation.stop();
}

void MainWindow::on_simulation_duration_input_editingFinished() {
    if (this->simulation.is_running)
        this->simulation.stop();

    this->simulation.set_duration(this->ui->simulation_duration_input->value());
    simulation.send_config();
}

void MainWindow::on_pid_kp_input_editingFinished() {
    this->simulation.pid->set_kp(this->ui->pid_kp_input->value());
    simulation.send_config();
}

void MainWindow::on_pid_ti_input_editingFinished() {
    this->simulation.pid->set_ti(this->ui->pid_ti_input->value());
    simulation.send_config();
}

void MainWindow::on_pid_td_input_editingFinished() {
    this->simulation.pid->set_td(this->ui->pid_td_input->value());
    simulation.send_config();
}


void MainWindow::on_generator_amplitude_input_editingFinished() {
    this->simulation.generator->set_amplitude(this->ui->generator_amplitude_input->value());
    simulation.send_config();
}

void MainWindow::on_generator_frequency_input_editingFinished() {
    this->simulation.generator->set_frequency(this->ui->generator_frequency_input->value());
    simulation.send_config();
}

void MainWindow::on_generator_infill_input_editingFinished() {
    this->simulation.generator->set_infill(this->ui->generator_infill_input->value());
    simulation.send_config();
}

void MainWindow::on_generator_generatortype_input_currentIndexChanged(int index) {
    this->simulation.generator->set_type(static_cast<GeneratorType>(index));
    simulation.send_config();
}

/*
void MainWindow::on_arx_noise_input_editingFinished()
{
    this->simulation.arx->set_noise(this->ui->arx_noise_input->value());
}

void MainWindow::on_arx_noisetype_input_currentIndexChanged(int index)
{
    this->simulation.arx->set_noise_type(static_cast<NoiseType>(index));
}

void MainWindow::on_arx_delay_input_editingFinished()
{
    this->simulation.arx->set_delay(this->ui->arx_delay_input->value());
}

void MainWindow::on_arx_b_input_editingFinished()
{
    QString arg1 = this->ui->arx_b_input->text();
    QStringList b_values = arg1.split(",");
    std::vector<float> b;
    for (const QString &value : b_values) {
        if (value.isEmpty())
            continue;
        try {
            b.push_back(value.toFloat());
        } catch (const std::exception &e) {
            qDebug() << e.what();
        }
    }

    this->simulation.arx->set_b(b);
}

void MainWindow::on_arx_a_input_editingFinished()
{
    QString arg1 = this->ui->arx_a_input->text();

    QStringList a_values = arg1.split(",");
    std::vector<float> a;
    for (const QString &value : a_values) {
        if (value.isEmpty())
            continue;
        try {
            a.push_back(value.toFloat());
        } catch (const std::exception &e) {
            qDebug() << e.what();
        }
    }

    this->simulation.arx->set_a(a);
}
*/
void MainWindow::on_simulation_reset_button_clicked()
{
    emit this->simulation.reset();
}

void MainWindow::on_simulation_interval_input_editingFinished() {
    if (this->simulation.is_running)
        this->simulation.stop();

    this->simulation.set_interval(this->ui->simulation_interval_input->value());
    simulation.send_config();
}






void MainWindow::on_radioButton_toggled(bool checked)
{
    this->simulation.pid->set_integral_mode_pullout(!checked);
     simulation.send_config();
}
void MainWindow::openArxDialog()
{
    qDebug() << "Przed otwarciem dialogu";
    ArxChangeParameters dialog(this);
     dialog.exec();
        QTimer::singleShot(200, []() {
        Simulation::get_instance().send_arx_config();
        });

        simulation.send_arx_config();




}


void MainWindow::on_btnPolacz_clicked()
{
    if (clientSocket && clientSocket->state() == QAbstractSocket::ConnectedState)
    {
        rozlaczKlienta();
        return;
    }

    if (server && server->isListening())
    {
        zatrzymajSerwer();
        return;
    }

    serwer dlg(this);
    if (dlg.exec() != QDialog::Accepted)
        return;

    QString ip = dlg.getIP();
    quint16 port = dlg.getPort();
    QString tryb = dlg.getTryb();

    ui->ip->setText("IP: " + ip);
    ui->port->setText("Port: " + QString::number(port));

    if (tryb == "klient")
    { simulation.network=true;
        simulation.isServer=false;

        if (clientSocket)
        {
            clientSocket->disconnectFromHost();
            clientSocket->deleteLater();
        }

        clientSocket = new QTcpSocket(this);
        ustawPolaczeniaKlienta();

        // Anuluj poprzedni timer jeśli istniał
        if (polaczenieTimer)
        {
            polaczenieTimer->stop();
            polaczenieTimer->deleteLater();
        }

        polaczenieTimer = new QTimer(this);
        polaczenieTimer->setSingleShot(true);
        connect(polaczenieTimer, &QTimer::timeout, this, [=]() {
            if (clientSocket && clientSocket->state() != QAbstractSocket::ConnectedState)
            {
                ui->Status->setText("Nie udało się połączyć z serwerem (timeout)");
                clientSocket->abort(); // przerywa próbę połączenia
                clientSocket->deleteLater();
                clientSocket = nullptr;
                ui->btnPolacz->setText("POŁĄCZ");
                ui->pid_kp_input->setEnabled(true);
                ui->pid_td_input->setEnabled(true);
                ui->pid_ti_input->setEnabled(true);
                ui->radioButton->setEnabled(true);
            }
        });
        polaczenieTimer->start(5000); // 5 sekund

        clientSocket->connectToHost(ip, port);
        ui->Status->setText("Łączenie z serwerem...");
        ui->pid_kp_input->setEnabled(false);
        ui->pid_td_input->setEnabled(false);
        ui->pid_ti_input->setEnabled(false);
        ui->radioButton->setEnabled(false);
        ui->btnPolacz->setText("ROZŁĄCZ");
    }

    else if (tryb == "serwer")
    {
        simulation.network=true;
        simulation.isServer=true;
        if (server)
        {
            server->close();
            server->deleteLater();
        }

        server = new QTcpServer(this);
        ustawPolaczeniaSerwera();

        if (server->listen(QHostAddress::AnyIPv4, port))
        {
            ui->Status->setText("Serwer nasłuchuje na porcie\n " + QString::number(port));
            ui->btnPolacz->setText("ROZŁĄCZ");
            ui->ArxButton->setEnabled(false);
        }
        else
        {
            ui->Status->setText("Błąd serwera: " + server->errorString());
            ui->ArxButton->setEnabled(true);
        }
    }
}

void MainWindow::ustawPolaczeniaKlienta()
{
    connect(clientSocket, &QTcpSocket::connected, this, &MainWindow::przyPolaczeniuKlienta);
    connect(clientSocket, &QTcpSocket::disconnected, this, &MainWindow::przyRozlaczeniuKlienta);
    connect(clientSocket, QOverload<QAbstractSocket::SocketError>::of(&QTcpSocket::errorOccurred),
            this, &MainWindow::bladPolaczeniaKlienta);
}

void MainWindow::ustawPolaczeniaSerwera()
{
    connect(server, &QTcpServer::newConnection, this, &MainWindow::nowePolaczenieNaSerwerze);
}
void MainWindow::przyPolaczeniuKlienta()
{
    if (polaczenieTimer)
    {
        polaczenieTimer->stop();
        polaczenieTimer->deleteLater();
        polaczenieTimer = nullptr;
    }

    ui->Status->setText("Połączono z serwerem");
    simulation.initialize_udp_receiver();


}


void MainWindow::przyRozlaczeniuKlienta()
{
    QString statusText;

    if (clientSocket && clientSocket->state() == QAbstractSocket::UnconnectedState)
    {
        statusText = "Połączenie z serwerem zostało przerwane\n (serwer zamknięty)";
    }
    else
    {
        statusText = "Rozłączono z serwerem";
    }
    ui->ip->setText("");
    ui->port->setText("");
    ui->Status->setText(statusText);
    //clientSocket->deleteLater();
   // clientSocket = nullptr;
    ui->pid_kp_input->setEnabled(true);
    ui->pid_td_input->setEnabled(true);
    ui->pid_ti_input->setEnabled(true);
    ui->radioButton->setEnabled(true);
    ui->btnPolacz->setText("POŁĄCZ");
}
void MainWindow::bladPolaczeniaKlienta(QAbstractSocket::SocketError blad)
{
    Q_UNUSED(blad)

    if (polaczenieTimer)
    {
        polaczenieTimer->stop();
        polaczenieTimer->deleteLater();
        polaczenieTimer = nullptr;
    }

    ui->Status->setText("Błąd połączenia: " + clientSocket->errorString());
    ui->pid_kp_input->setEnabled(true);
    ui->pid_td_input->setEnabled(true);
    ui->pid_ti_input->setEnabled(true);
    ui->radioButton->setEnabled(true);
    ui->btnPolacz->setText("POŁĄCZ");
}

void MainWindow::nowePolaczenieNaSerwerze()
{
    clientConnection = server->nextPendingConnection();

    QString ip = clientConnection->peerAddress().toString();
    ip.remove('[').remove(']');
    if (ip == "::1") ip = "127.0.0.1";

    connect(clientConnection, &QTcpSocket::readyRead, this, [=]() {
        QByteArray data = clientConnection->readAll();
        QString message = QString::fromUtf8(data).trimmed();

        if (message == "DISCONNECT")
        {
            ui->Status->setText("Klient " + ip + " poinformował o rozłączeniu");
            qDebug() << "Klient " << ip << " poinformował o rozłączeniu";
        }
        else
        {
            qDebug() << "Odebrano wiadomość od klienta:" << message;
        }
    });

    connect(clientConnection, &QTcpSocket::disconnected, this, [=]() {
        ui->Status->setText("Klient " + ip + " został rozłączony");
        qDebug() << "Klient " << ip << " został rozłączony";
        clientConnection->deleteLater();
        clientConnection = nullptr;
    });

    ui->Status->setText("Nowe połączenie od " + ip);
    qDebug() << "Nowe połączenie od" << ip;
}

void MainWindow::rozlaczKlienta()
{
    clientSocket->write("DISCONNECT\n");
    clientSocket->flush();
    QThread::msleep(100);

    clientSocket->disconnectFromHost();
    ui->Status->setText("Rozłączono z serwerem (na żądanie)");
    ui->pid_kp_input->setEnabled(true);
    ui->pid_td_input->setEnabled(true);
    ui->pid_ti_input->setEnabled(true);
    ui->radioButton->setEnabled(true);
    ui->btnPolacz->setText("POŁĄCZ");
}

void MainWindow::zatrzymajSerwer()
{
    if (clientConnection)
    {
        clientConnection->write("DISCONNECT\n");
        clientConnection->flush();
        QThread::msleep(100);
        clientConnection->disconnectFromHost();
        clientConnection = nullptr;
    }
    ui->ip->setText("");
    ui->port->setText("");
    server->close();
    ui->ArxButton->setEnabled(true);
    ui->Status->setText("Serwer zatrzymany");
    ui->btnPolacz->setText("POŁĄCZ");
}
