#ifndef SIMULATION_H
#define SIMULATION_H

#include <qudpsocket.h>

#include <QMainWindow>
#include <QObject>
#include "arx.h"
#include "generator.h"
#include "pid.h"


enum class PacketType : quint8 {
    StartSignal = 0,
    Generator = 1,
    ClientResponse = 2,
    ResetCommand = 3,
    ConfigServer = 4,  // konfiguracja PID i generatora (wysyłana przez serwer)
    ConfigARX =5     // konfiguracja ARX (wysyłana przez klienta)
};


struct ConfigARXPacket {
    PacketType type = PacketType::ConfigARX;
    std::vector<float> a;
    std::vector<float> b;
    size_t delay;
    float noise;
    NoiseType noise_type;

    friend QDataStream& operator<<(QDataStream& out, const ConfigARXPacket& data) {
        out << static_cast<quint8>(data.type)
        << static_cast<quint32>(data.a.size());
        for (float val : data.a) out << val;
        out << static_cast<quint32>(data.b.size());
        for (float val : data.b) out << val;
        out << static_cast<quint32>(data.delay)
            << data.noise
            << static_cast<qint32>(data.noise_type);
        return out;
    }

    friend QDataStream& operator>>(QDataStream& in, ConfigARXPacket& data) {
        quint8 typeByte;
        in >> typeByte;
        data.type = static_cast<PacketType>(typeByte);

        quint32 aSize;
        in >> aSize;
        data.a.resize(aSize);
        for (quint32 i = 0; i < aSize; ++i) in >> data.a[i];

        quint32 bSize;
        in >> bSize;
        data.b.resize(bSize);
        for (quint32 i = 0; i < bSize; ++i) in >> data.b[i];

        quint32 delay;
        qint32 noiseTypeInt;
        in >> delay >> data.noise >> noiseTypeInt;

        data.delay = delay;
        data.noise_type = static_cast<NoiseType>(noiseTypeInt);
        return in;
    }
};



struct ConfigServerPacket {
    PacketType type = PacketType::ConfigServer;
    int interval;
    float duration;
    float pid_kp;
    float pid_ti;
    float pid_td;
    bool pid_ti_pullout;
    float generator_amplitude;
    float generator_frequency;
    GeneratorType generator_type;

    friend QDataStream& operator<<(QDataStream& out, const ConfigServerPacket& data) {
        out << static_cast<quint8>(data.type)
        << data.interval
        << data.duration
        << data.pid_kp << data.pid_ti << data.pid_td
        << data.pid_ti_pullout
        << data.generator_amplitude << data.generator_frequency
        << static_cast<qint32>(data.generator_type);
        return out;
    }

    friend QDataStream& operator>>(QDataStream& in, ConfigServerPacket& data) {
        quint8 typeByte;
        qint32 generatorTypeInt;
        in >> typeByte;
        data.type = static_cast<PacketType>(typeByte);
        in >> data.interval
            >> data.duration
            >> data.pid_kp >> data.pid_ti >> data.pid_td
            >> data.pid_ti_pullout
            >> data.generator_amplitude >> data.generator_frequency
            >> generatorTypeInt;
        data.generator_type = static_cast<GeneratorType>(generatorTypeInt);
        return in;
    }
};




struct ClientResponsePacket {
    PacketType type = PacketType::ClientResponse;
    size_t tick;
    float arx_output;
    float zaklucenie;

    friend QDataStream& operator<<(QDataStream& out, const ClientResponsePacket& data) {
        out << static_cast<quint8>(data.type)
        << data.tick
        << data.arx_output
        << data.zaklucenie;
        return out;
    }

    friend QDataStream& operator>>(QDataStream& in, ClientResponsePacket& data) {
        quint8 typeByte;
        in >> typeByte;
        data.type = static_cast<PacketType>(typeByte);
        in >> data.tick >> data.arx_output >> data.zaklucenie;
        return in;
    }
};



struct GeneratorPacket {
    PacketType type = PacketType::Generator;
    size_t tick;
    float generator;
    float p;
    float i;
    float d;
    float error;
    float arx_output;
    float pid_output;

    friend QDataStream& operator<<(QDataStream& out, const GeneratorPacket& data) {
        out << static_cast<quint8>(data.type)
        << data.tick
        << data.generator
        << data.p
        << data.i
        << data.d
        << data.error
        << data.arx_output
        << data.pid_output;
        return out;
    }

    friend QDataStream& operator>>(QDataStream& in, GeneratorPacket& data) {
        quint8 typeByte;
        in >> typeByte;
        data.type = static_cast<PacketType>(typeByte);
        in >> data.tick
            >> data.generator
            >> data.p
            >> data.i
            >> data.d
            >> data.error
            >> data.arx_output
            >> data.pid_output;
        return in;
    }
};



enum class ChartPosition {
    top,
    middle,
    bottom,
};

struct Point
{
    float x;
    float y;
};

struct SimulationFrame
{
    size_t tick;
    float geneartor_output;

    float p;
    float i;
    float d;
    float pid_output;

    float error;

    float arx_output;
    float noise;



};

class Simulation : public QObject
{
    Q_OBJECT
public:
    void initialize_udp_receiver();  // deklaracja

    //
    //udp
    //

    void send_arx_config();
    void send_config();
    bool simulation_started_by_udp = false;
 bool client_data_received = false;

    bool network =false;
    bool isServer = false;
    QUdpSocket udpSocket;
    QByteArray buffer;
    //

    static Simulation &get_instance();

    void start();
    void stop();

    void set_ticks_per_second(float ticks_per_second);
    void set_duration(float duration);

    size_t get_tick();
    float get_ticks_per_second() const;

    void increment_tick();

    float durration{0};

    bool is_running{false};

    size_t duration_timer_id{0};

    void reset();

    void set_interval(int interval);
    int get_interval() const;

    std::unique_ptr<PID> pid;
    std::unique_ptr<Generator> generator;
    std::unique_ptr<ARX> arx;

    std::list<SimulationFrame> frames{};

    std::vector<std::byte> serialize();
    void deserialize(std::vector<std::byte> data);


    void simulate_local();
    void simulate_server();
    void simulate_client();




signals:
      void communication_status(bool ok);
    void simulation_start();
    void simulation_stop();

    void reset_chart();
    void update_chart();
    void add_series(QString series_name, float y, ChartPosition position);

protected:
    void timerEvent(QTimerEvent *event) override;

private:
    void simulate();
    void emit_frame_to_chart(const SimulationFrame& frame);

    QUdpSocket ARXudpSocketResponse;
    QUdpSocket udpSocketResponse; // nowy socket dla odpowiedzi od klienta
    float last_arx_from_client = 0.0f; // tutaj zapiszemy wartość z klienta
    float last_noise_from_client = 0.0f;
    float ticks_per_second{60};
    size_t tick{0};
    size_t timer_id{0};
    int interval{100};

    explicit Simulation(QObject *parent = nullptr);
    ~Simulation();

signals:


    void config_arx_received(const ConfigARXPacket&);
    void config_server_received(const ConfigServerPacket&);


};






#endif // SIMULATION_H
