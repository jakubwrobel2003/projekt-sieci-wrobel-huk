#include "simulation.h"


void Simulation::send_arx_config() {
    if (!network || isServer) {
        qDebug() << "[CLIENT] NIE wysyłam – warunki niespełnione";
        return;
    }

    ConfigARXPacket packet{
        .a = arx->get_a(),
        .b = arx->get_b(),
        .delay = arx->get_delay(),
        .noise = arx->get_noise(),
        .noise_type = arx->get_noise_type()
    };

    QByteArray data;
    QDataStream out(&data, QIODevice::WriteOnly);
    out.setVersion(QDataStream::Qt_6_0);
    out << packet;


    udpSocket.writeDatagram(data, QHostAddress(remoteIp), PORT_SERWERA);


    //udpSocket.writeDatagram(data, QHostAddress(remoteIp), 1222);
    qDebug() << "[CLIENT] Wysłano ConfigARX na port 1222";
}


void Simulation::deinitialize(bool resetSimulation)
{
    qDebug() << "[UDP] Deinitializacja komunikacji UDP";

    // Zatrzymaj timer, jeśli działa
    if (this->is_running) {
        this->stop();
    }

    // Odłącz sygnały (jeśli były podpięte)
    disconnect(&udpSocket, nullptr, nullptr, nullptr);

    // Zamknij socket, jeśli był zbindowany
    if (udpSocket.state() == QAbstractSocket::BoundState) {
        udpSocket.close();
        qDebug() << "[UDP] Zamknięto socket UDP";
    }

    // Wyzeruj flagi sieciowe
    this->network = false;
    this->isServer = false;
    this->simulation_started_by_udp = false;
    this->client_data_received = false;
    //this->client_initialized = false;

    // Wyzeruj adres IP i porty (opcjonalnie)
    this->remoteIp = "127.0.0.1";
    this->PORT_KLIENTA = 1234;
    this->PORT_SERWERA = 1235;

    // Resetuj stan symulacji i wykresów (opcjonalnie)
    if (resetSimulation) {
        this->reset();
    }

    emit communication_status(false);  // do GUI: rozłączono
}




void Simulation::send_config()
{
    if (!network || !isServer) return;

    ConfigServerPacket config {
        .interval = this->interval,
        .duration = this->durration,
        .pid_kp = pid->get_kp(),
        .pid_ti = pid->get_ti(),
        .pid_td = pid->get_td(),
        .pid_ti_pullout = pid->get_integral_mode_pillout(),  // ✅ poprawiona literówka
        .generator_amplitude = generator->get_amplitude(),
        .generator_frequency = generator->get_frequency(),
        .generator_type = generator->get_type()
    };

    QByteArray data;
    QDataStream out(&data, QIODevice::WriteOnly);
    out.setVersion(QDataStream::Qt_6_0);
    out << config;

    udpSocket.writeDatagram(data, QHostAddress(remoteIp), PORT_KLIENTA);

      // lub dynamicznie np. clientIP
    qDebug() << "[SERVER] Wysłano ConfigServerPacket:"
             << "interval =" << config.interval
             << "duration =" << config.duration
             << "Kp =" << config.pid_kp
             << "Ti =" << config.pid_ti
             << "Td =" << config.pid_td;
}





Simulation::Simulation(QObject *parent)
    : QObject{parent}
{
    this->pid = std::make_unique<PID>();
    this->generator = std::make_unique<Generator>();
    this->arx = std::make_unique<ARX>();




    //

}
void Simulation::initialize_udp_receiver()
{

    quint16 port = isServer ? PORT_SERWERA : PORT_KLIENTA;

    if (!udpSocket.bind(QHostAddress(remoteIp), port,

                        QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint)) {
       // qWarning() << "[UDP] Nie udało się zbindować portu"   ":" << udpSocket.errorString();
    } else {

        qDebug() << "[UDP] Zbindowano port" << port
                 << (isServer ? "(serwer)" : "(klient)");


        connect(&udpSocket, &QUdpSocket::readyRead, this, [this]() {
            if (!this->network) return;
            if (this->isServer) {
                this->receive_from_client();
            } else {
                this->simulate_client();
            }
        });
    }
}

void Simulation::receive_from_client()
{
    while (udpSocket.hasPendingDatagrams()) {
        QByteArray buffer;
        QHostAddress sender;
        quint16 senderPort;
        buffer.resize(udpSocket.pendingDatagramSize());
        udpSocket.readDatagram(buffer.data(), buffer.size(), &sender, &senderPort);

        if (buffer.isEmpty()) {
            qWarning() << "[SERVER] Odebrano pusty pakiet – pominięty";
            continue;
        }

        QDataStream peek(&buffer, QIODevice::ReadOnly);
        peek.setVersion(QDataStream::Qt_6_0);
        quint8 typeByte;
        peek >> typeByte;
        PacketType type = static_cast<PacketType>(typeByte);

        qDebug() << "[SERVER] Odebrano pakiet typu:" << static_cast<int>(type);


        if (type == PacketType::ClientResponse) {
            QDataStream in(&buffer, QIODevice::ReadOnly);
            in.setVersion(QDataStream::Qt_6_0);
            in.device()->seek(0);
            ClientResponsePacket response;
            in >> response;

            this->last_arx_from_client = response.arx_output;
            this->last_noise_from_client = response.zaklucenie;
            qDebug() << "[SERVER] Odebrano odpowiedź od klienta – tick:" << response.tick;
        }
        else if (type == PacketType::ConfigARX) {
            QDataStream in(&buffer, QIODevice::ReadOnly);
            in.setVersion(QDataStream::Qt_6_0);
            in.device()->seek(0);
            ConfigARXPacket packet;
            in >> packet;

            arx->set_a(packet.a);
            arx->set_b(packet.b);
            arx->set_delay(packet.delay);
            arx->set_noise(packet.noise);
            arx->set_noise_type(packet.noise_type);

            emit config_arx_received(packet);
            qDebug() << "[SERVER] Odebrano ConfigARX od klienta";
        }
        else {
            qDebug() << "[SERVER] Nieznany typ pakietu:" << static_cast<int>(type);

        }
    }
}







Simulation &Simulation::get_instance()
{
    static Simulation instance;
    return instance;
}

Simulation::~Simulation()
{
    this->killTimer(this->timer_id);
}

float Simulation::get_ticks_per_second() const
{
    return this->ticks_per_second;
}

void Simulation::increment_tick()
{
    this->tick++;
}

size_t Simulation::get_tick()
{
    return this->tick;
}

template<typename T>
void memcopy_s(void *dest, const T &src, size_t size = 1)
{
    std::memcpy(dest, &src, sizeof(T) * size);
}


void Simulation::emit_frame_to_chart(const SimulationFrame& frame)
{
    emit this->add_series("PID P", frame.p, ChartPosition::top);
    emit this->add_series("PID I", frame.i, ChartPosition::top);
    emit this->add_series("PID D", frame.d, ChartPosition::top);
    emit this->add_series("PID Output", frame.pid_output, ChartPosition::top);
    emit this->add_series("Generator Output", frame.geneartor_output, ChartPosition::middle);
    emit this->add_series("Error", frame.error, ChartPosition::middle);
    emit this->add_series("ARX Output", frame.arx_output, ChartPosition::bottom);
    emit this->add_series("Noise", frame.noise, ChartPosition::middle);
    emit this->update_chart();
}



void Simulation::simulate_local() {
    static float error = 0;
    static float arx_output = 0;
    static float pid_output = 0;
    static float generator = 0;

    SimulationFrame frame;
    const size_t tick = this->get_tick();
    const float time = tick / this->ticks_per_second;

    generator = this->generator->run(time);
    error = generator - arx_output;
    pid_output = this->pid->run(error);
    arx_output = this->arx->run(pid_output);

    frame.tick = tick;
    frame.geneartor_output = generator;
    frame.p = this->pid->proportional_part;
    frame.i = this->pid->integral_part;
    frame.d = this->pid->derivative_part;
    frame.pid_output = pid_output;
    frame.error = error;
    frame.arx_output = arx_output;
    frame.noise = this->arx->noise_part;

    this->frames.push_back(frame);
    emit_frame_to_chart(frame);
    this->tick++;
}

void Simulation::simulate_server() {
    const size_t tick = this->get_tick();
    const float time = tick / this->ticks_per_second;

    // ODBIÓR OD KLIENTA
    while (udpSocket.hasPendingDatagrams()) {
        QByteArray buffer;
        QHostAddress sender;
        quint16 senderPort;
        buffer.resize(udpSocket.pendingDatagramSize());
        udpSocket.readDatagram(buffer.data(), buffer.size(), &sender, &senderPort);

        QDataStream peek(&buffer, QIODevice::ReadOnly);
        peek.setVersion(QDataStream::Qt_6_0);
        quint8 typeByte;
        peek >> typeByte;
        PacketType type = static_cast<PacketType>(typeByte);

        qDebug() << "[SERVER] Odebrano pakiet typu:" << static_cast<int>(typeByte);


        // 🔴 FILTR: jeśli serwer odbiera swój własny pakiet – ignoruj
        if (sender.toString() == remoteIp && senderPort == 1234 &&
            (type == PacketType::ConfigServer || type == PacketType::StartSignal || type == PacketType::ResetCommand)) {
            qDebug() << "[SERVER] Ignoruję własny pakiet typu" << static_cast<int>(type);
            continue;
        }

        qDebug() << "[SERVER] Typ pakietu (typeByte):" << static_cast<int>(typeByte);










        if (static_cast<PacketType>(typeByte) == PacketType::ClientResponse) {
            QDataStream in(&buffer, QIODevice::ReadOnly);  // <-- TUTAJ dodajesz strumień
            in.setVersion(QDataStream::Qt_6_0);

            ClientResponsePacket response;
            in >> response;

            this->last_arx_from_client = response.arx_output;
            this->last_noise_from_client = response.zaklucenie;

            qDebug() << "[SERVER] Odebrano odpowiedź: tick=" << response.tick;
            continue;
        }

    }

    // OBLICZENIA
    float generator = this->generator->run(time);
    float error = generator - last_arx_from_client;
    float pid_output = pid->run(error);

    GeneratorPacket packet{
        .type = PacketType::Generator,
        .tick = tick,
        .generator = generator,
        .p = pid->proportional_part,
        .i = pid->integral_part,
        .d = pid->derivative_part,
        .error = error,
        .arx_output = last_arx_from_client,
        .pid_output = pid_output
    };

    QByteArray datagram;
    QDataStream out(&datagram, QIODevice::WriteOnly);
    out.setVersion(QDataStream::Qt_6_0);
    out << packet;

    udpSocket.writeDatagram(datagram, QHostAddress(remoteIp), PORT_KLIENTA);

    qDebug() << "[SERVER] Wysłano tick=" << tick;

    SimulationFrame frame;
    frame.tick = tick;
    frame.geneartor_output = generator;
    frame.p = packet.p;
    frame.i = packet.i;
    frame.d = packet.d;
    frame.pid_output = packet.pid_output;
    frame.error = packet.error;
    frame.arx_output = last_arx_from_client;
    frame.noise = last_noise_from_client;

    this->frames.push_back(frame);
    emit_frame_to_chart(frame);
    this->tick++;
}



void Simulation::simulate_client() {
    while (udpSocket.hasPendingDatagrams()) {
        QByteArray buffer;
        QHostAddress sender;
        quint16 senderPort;
        buffer.resize(udpSocket.pendingDatagramSize());
        udpSocket.readDatagram(buffer.data(), buffer.size(), &sender, &senderPort);

        QDataStream in(&buffer, QIODevice::ReadOnly);
        in.setVersion(QDataStream::Qt_6_0);

        quint8 typeByte;
        in >> typeByte;
        PacketType type = static_cast<PacketType>(typeByte);

        qDebug() << "[CLIENT] Odebrano pakiet typu:" << static_cast<int>(type)
                 << "z" << sender.toString() << ":" << senderPort;
        qDebug() << "[CLIENT] Odebrano bajty:" << buffer.toHex();




        if (type == PacketType::ConfigServer) {
            in.device()->seek(0);
            ConfigServerPacket packet;
            in >> packet;

            pid->set_kp(packet.pid_kp);
            pid->set_ti(packet.pid_ti);
            pid->set_td(packet.pid_td);
            pid->set_integral_mode_pullout(packet.pid_ti_pullout);
            generator->set_amplitude(packet.generator_amplitude);
            generator->set_frequency(packet.generator_frequency);
            generator->set_type(packet.generator_type);
            this->interval = packet.interval;
            this->durration = packet.duration;

            emit config_server_received(packet);
            qDebug() << "[CLIENT] Odebrano konfigurację serwera";
            continue;
        }

        if (type == PacketType::StartSignal) {
            qDebug() << "[CLIENT] Odebrano StartSignal – uruchamiam timer";
            if (!this->is_running) {
                const int interval_ms = std::max(this->interval, 30);
                this->timer_id = this->startTimer(interval_ms);
                this->is_running = true;
                emit this->simulation_start();
            }
            continue;
        }

        if (type == PacketType::ResetCommand) {
            qDebug() << "[CLIENT] Odebrano RESET – wykonuję reset ARX";
            this->arx->reset();
            this->tick = 0;
            this->frames.clear();
            emit this->reset_chart();
            continue;
        }

        if (type != PacketType::Generator) {
            qWarning() << "[CLIENT] Odrzucono pakiet: nie Generator";
            continue;
        }

        // GeneratorPacket
        in.device()->seek(0);
        GeneratorPacket packet;
        in >> packet;

        if (packet.tick == 0) {
            qDebug() << "[CLIENT] Reset ARX (tick == 0)";
            this->arx->reset();
            this->frames.clear();
            emit this->reset_chart();
        }

        float noise = this->arx->noise_part;
        float arx_output = this->arx->run(packet.pid_output);
        float arx_with_noise = arx_output + noise;

        SimulationFrame frame{
            .tick = packet.tick,
            .geneartor_output = packet.generator,
            .p = packet.p,
            .i = packet.i,
            .d = packet.d,
            .pid_output = packet.pid_output,
            .error = packet.error,
            .arx_output = arx_output,
            .noise = noise
        };

        this->frames.push_back(frame);
        emit_frame_to_chart(frame);
        this->tick = packet.tick;

        // Odpowiedź do serwera
        ClientResponsePacket response{
            .type = PacketType::ClientResponse,
            .tick = packet.tick,
            .arx_output = arx_with_noise,
            .zaklucenie = noise
        };

        QByteArray responseData;
        QDataStream out(&responseData, QIODevice::WriteOnly);
        out.setVersion(QDataStream::Qt_6_0);
        out << response;
        udpSocket.writeDatagram(responseData, sender, senderPort); // ✅ działało, bo odpowiadał dokładnie na port źródła

        qDebug() << "[CLIENT] Wysłano odpowiedź: tick=" << response.tick;
    }
}







void Simulation::simulate() {
    if (!network) {
        simulate_local();
    } else if (isServer) {
        simulate_server();
    } else {
        simulate_client();
    }
}





   /* SimulationFrame frame{
        .tick = tick,
        .geneartor_output = generator,
        .p = this->pid->proportional_part,
        .i = this->pid->integral_part,
        .d = this->pid->derivative_part,
        .pid_output = pid_output,
        .error = error,
        .arx_output = arx_output,
        .noise = this->arx->noise_part,
    };

    this->frames.push_back(frame);

    emit this->add_series("I", this->pid->integral_part, ChartPosition::top);
    emit this->add_series("D", this->pid->derivative_part, ChartPosition::top);
    emit this->add_series("P", this->pid->proportional_part, ChartPosition::top);
    emit this->add_series("PID", pid_output, ChartPosition::top);

    emit this->add_series("Generator", generator, ChartPosition::bottom);
    ;
    emit this->add_series("Error", error, ChartPosition::middle);

    emit this->add_series("ARX", arx_output, ChartPosition::bottom);
    emit this->add_series("Noise", this->arx->noise_part, ChartPosition::middle);

    emit this->update_chart();*/


void Simulation::set_ticks_per_second(float ticks_per_second)
{
    this->ticks_per_second = ticks_per_second;
}

void Simulation::timerEvent(QTimerEvent *event)
{
    this->simulate();
}
void Simulation::start()
{
    const int interval_ms = std::max(this->interval, 30);
    qDebug() << "[SIMULATION] start() wywołane, isServer=" << isServer;

    if (this->is_running) {
        qDebug() << "[SIMULATION] Już działa, pomijam start";
        return;
    }

    this->timer_id = this->startTimer(interval_ms);
    this->is_running = true;
    qDebug() << "[SIMULATION] startTimer OK, interval =" << interval_ms;

    // 🔴 TYLKO serwer wysyła StartSignal do klienta (na ten sam port: 1234)
    if (network && isServer) {
        QByteArray startPacket;
        QDataStream out(&startPacket, QIODevice::WriteOnly);
        out.setVersion(QDataStream::Qt_6_0);
        out << static_cast<quint8>(PacketType::StartSignal);

        udpSocket.writeDatagram(startPacket, QHostAddress(remoteIp), PORT_KLIENTA);
         qDebug() << "[SERVER] Wysłano StartSignal do klienta (na 127.0.0.1:1234)";
    }

    emit this->simulation_start();  // może być użyte do GUI
}


void Simulation::reset()
{
    this->stop();

    this->tick = 0;
    this->is_running = false;

    emit this->reset_chart();

    this->pid->reset();
    this->arx->reset();
    this->frames.clear();
    if (network && isServer) {
        // Wyślij ResetCommand do klienta
        QByteArray resetPacket;
        QDataStream out(&resetPacket, QIODevice::WriteOnly);
        out.setVersion(QDataStream::Qt_6_0);

        out << static_cast<quint8>(PacketType::ResetCommand);

        udpSocket.writeDatagram(resetPacket, QHostAddress(remoteIp), PORT_KLIENTA);
          qDebug() << "[SERVER] Wysłano RESET do klienta";
    }
}

void Simulation::stop()
{
    this->killTimer(this->timer_id);
    this->is_running = false;

    emit this->simulation_stop();
}

void Simulation::set_duration(float duration)
{
    this->durration = duration;
}

void Simulation::set_interval(int interval)
{
    this->interval = interval;
}

int Simulation::get_interval() const
{
    return this->interval;
}

struct SimulationDeserialized
{
    // simulation
    int interval;    // [0:3]
    float durration; // [4:7]

    // pid
    float pid_kp; // [8:11]
    float pid_ti; // [12:15]
    float pid_td; // [16:19]

    // generator
    float generator_amplitude;    // [20:23]
    float generator_frequency;    // [24:27]
    GeneratorType generator_type; // [28:31]

    // arx
    float arx_noise;          // [32:35]
    NoiseType arx_noise_type; // [36:39]
    size_t arx_delay;         // [40:43]
};

std::vector<std::byte> Simulation::serialize()
{
    constexpr size_t data_size = sizeof(SimulationDeserialized);
    const size_t vector_sizes = sizeof(float) * this->arx->get_a().size()
                                + sizeof(float) * this->arx->get_b().size() + (sizeof(size_t) * 2);

    std::vector<std::byte> data(data_size + vector_sizes);

    SimulationDeserialized deserialized{
        this->interval,
        this->durration,
        this->pid->get_kp(),
        this->pid->get_ti(),
        this->pid->get_td(),
        this->generator->get_amplitude(),
        this->generator->get_frequency(),
        this->generator->get_type(),
        this->arx->get_noise(),
        this->arx->get_noise_type(),
        this->arx->get_delay(),
    };

    std::byte *data_ptr = data.data();

    std::memcpy(data_ptr, &deserialized, data_size);
    data_ptr += data_size;

    size_t a_size = this->arx->get_a().size();
    memcpy(data_ptr, &a_size, sizeof(size_t));
    data_ptr += sizeof(size_t);

    qDebug() << "a_size: " << a_size;

    memcpy(data_ptr, this->arx->get_a().data(), sizeof(float) * a_size);
    data_ptr += sizeof(float) * this->arx->get_a().size();

    size_t b_size = this->arx->get_b().size();
    qDebug() << "b_size" << b_size;
    memcpy(data_ptr, &b_size, sizeof(size_t));
    data_ptr += sizeof(size_t);

    memcpy(data_ptr, this->arx->get_a().data(), sizeof(float) * this->arx->get_a().size());
    data_ptr += sizeof(float) * this->arx->get_a().size();

    qDebug() << "data size: " << data.size();

    return data;
}

void Simulation::deserialize(std::vector<std::byte> data)
{
    constexpr size_t data_size = sizeof(SimulationDeserialized);

    qDebug() << "data size: " << data.size();

    SimulationDeserialized deserialized;

    std::byte *data_ptr = data.data();

    std::memcpy(&deserialized, data_ptr, sizeof(SimulationDeserialized));
    data_ptr += sizeof(SimulationDeserialized);

    size_t a_size;
    memcpy(&a_size, data_ptr, sizeof(size_t));
    data_ptr += sizeof(size_t);

    qDebug() << "a_size: " << a_size;

    std::vector<float> a(a_size);
    memcpy(a.data(), data_ptr, sizeof(float) * a_size);
    data_ptr += sizeof(float) * a_size;

    size_t b_size;
    memcpy(&b_size, data_ptr, sizeof(size_t));
    data_ptr += sizeof(size_t);
    qDebug() << "b_size" << b_size;

    std::vector<float> b(b_size);
    memcpy(b.data(), data_ptr, sizeof(float) * b_size);

    this->set_interval(deserialized.interval);
    this->set_duration(deserialized.durration);
    this->pid->set_kp(deserialized.pid_kp);
    this->pid->set_ti(deserialized.pid_ti);
    this->pid->set_td(deserialized.pid_td);
    this->generator->set_amplitude(deserialized.generator_amplitude);
    this->generator->set_frequency(deserialized.generator_frequency);
    this->generator->set_type(deserialized.generator_type);
    this->arx->set_a(a);
    this->arx->set_b(b);
    this->arx->set_noise(deserialized.arx_noise);
    this->arx->set_noise_type(deserialized.arx_noise_type);
    this->arx->set_delay(deserialized.arx_delay);
}
