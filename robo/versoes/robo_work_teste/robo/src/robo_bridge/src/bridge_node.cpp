#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <geometry_msgs/msg/twist.hpp>
#include <glob.h>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <sstream>
#include <iomanip>

class BridgeNode : public rclcpp::Node{
public:
    BridgeNode(): Node("bridge_node"), serial_port_(-1), stop_sent_(true) {
        RCLCPP_INFO(
            this->get_logger(),
            "robo_bridge iniciado!"
        );

        cmd_vel_sub_ =this->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel",
                10,
                bind(
                    &BridgeNode::cmd_vel_callback,
                    this,
                    std::placeholders::_1
                )
            );
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/scan_raw",
            10
        );

        // IMU BNO055 (so com o firmware v2). Os sinais de yaw/giro dependem
        // da montagem da placa: validar girando o robo e comparando com a
        // odometria por LiDAR antes de usar a IMU na navegacao.
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 20);
        imu_frame_ = this->declare_parameter<std::string>("imu_frame", "base_link");
        imu_inverter_yaw_ = this->declare_parameter<bool>("imu_inverter_yaw", true);
        imu_inverter_giro_ = this->declare_parameter<bool>("imu_inverter_giro", false);

        // Vazio = procura o Arduino sozinho (by-id, ttyUSB*, ttyACM*).
        // Ex.: ros2 run robo_bridge bridge_node --ros-args -p porta_serial:=/dev/ttyUSB1
        porta_serial_param_ =
            this->declare_parameter<std::string>("porta_serial", "");

        open_serial();

        // Se o cabo sair (ou a porta mudar de ttyUSB0 para ttyUSB1), tenta
        // reabrir a cada 1 s. O firmware para os motores sozinho pelo
        // watchdog enquanto nao chegam comandos.
        reconnect_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&BridgeNode::check_serial_connection, this)
        );

        watchdog_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(50),
                std::bind(
                    &BridgeNode::check_command_timeout,
                    this
                )
            );

        serial_read_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            bind(&BridgeNode::read_serial, this)
        );
    }
   


    ~BridgeNode(){
        if (serial_port_ >= 0)
        {
            if (send_motor_command(0, 0))
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Comando STOP enviado antes de fechar a serial"
                );
            }
            else
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Falha ao enviar STOP antes de fechar a serial"
                );
            }

            close(serial_port_);

            RCLCPP_INFO(
                this->get_logger(),
                "Porta serial fechada"
            );
        }
    }


private:

    bool configure_serial(){
        struct termios tty;

        if (tcgetattr(serial_port_, &tty) != 0){
    
            RCLCPP_ERROR(
                this->get_logger(),
                "Erro ao ler configurações da serial: %s",
                strerror(errno)
            );

            return false;
        }
        cfmakeraw(&tty);

        cfsetispeed(&tty, B500000);
        cfsetospeed(&tty, B500000);

        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;

        tty.c_cflag |= CREAD | CLOCAL;
        tty.c_cflag &= ~CRTSCTS;

        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(serial_port_, TCSANOW, &tty) != 0){
            RCLCPP_ERROR(
                this->get_logger(),
                "Erro ao configurar serial: %s",
                strerror(errno)
            );

            return false;
        }
        
        tcflush(serial_port_, TCIOFLUSH);

        RCLCPP_INFO(
            this->get_logger(),
            "Serial configurada em 500000 8n1"
        );

        return true;
    }

    // ---------- Conexao / reconexao da serial ----------

    std::vector<std::string> serial_candidates() const {
        if (!porta_serial_param_.empty()) {
            return {porta_serial_param_};
        }

        // by-id primeiro: o nome nao muda quando o kernel troca ttyUSB0/1.
        std::vector<std::string> candidates;
        for (const char *pattern : {"/dev/serial/by-id/*", "/dev/ttyUSB*", "/dev/ttyACM*"}) {
            glob_t result{};
            if (glob(pattern, 0, nullptr, &result) == 0) {
                for (size_t i = 0; i < result.gl_pathc; ++i) {
                    candidates.emplace_back(result.gl_pathv[i]);
                }
            }
            globfree(&result);
        }
        return candidates;
    }

    bool open_serial() {
        const auto candidates = serial_candidates();
        if (candidates.empty()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Nenhuma porta serial encontrada (Arduino desconectado?). "
                "Tentando de novo a cada 1 s"
            );
            return false;
        }

        for (const auto &path : candidates) {
            const int fd = open(path.c_str(), O_RDWR | O_NOCTTY);
            if (fd < 0) {
                continue;
            }
            serial_port_ = fd;
            if (!configure_serial()) {
                close(fd);
                serial_port_ = -1;
                continue;
            }

            serial_path_ = path;
            serial_buffer_.clear();
            // Abrir a porta reinicia o Mega; da tempo do firmware subir
            // antes de cobrar dados (ver check_serial_connection).
            last_rx_time_ = std::chrono::steady_clock::now();
            opened_time_ = last_rx_time_;
            RCLCPP_INFO(
                this->get_logger(),
                "Porta serial aberta: %s",
                path.c_str()
            );
            return true;
        }

        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Nao foi possivel abrir nenhuma porta serial (%zu candidata(s)): %s",
            candidates.size(),
            strerror(errno)
        );
        return false;
    }

    void close_serial(const char *reason) {
        if (serial_port_ < 0) {
            return;
        }
        close(serial_port_);
        serial_port_ = -1;
        serial_buffer_.clear();
        // Sem serial nao ha como mandar STOP; o watchdog do firmware para os
        // motores. Marca como parado para nao gerar timeout repetido.
        stop_sent_ = true;
        RCLCPP_ERROR(
            this->get_logger(),
            "Serial %s perdida (%s). Motores parados pelo watchdog do firmware; "
            "tentando reconectar...",
            serial_path_.c_str(),
            reason
        );
    }

    static bool is_disconnect_error(int error) {
        return error == EIO || error == ENODEV || error == ENXIO || error == EBADF;
    }

    void check_serial_connection() {
        if (serial_port_ < 0) {
            if (open_serial()) {
                RCLCPP_INFO(this->get_logger(), "Serial reconectada");
            }
            return;
        }

        // Alguns desligamentos nao geram erro no read(): so param os dados.
        // O firmware manda status varias vezes por segundo, entao silencio
        // longo (depois do tempo de boot) significa conexao morta.
        constexpr auto BOOT_GRACE = std::chrono::seconds(5);
        constexpr auto RX_TIMEOUT = std::chrono::seconds(3);
        const auto now = std::chrono::steady_clock::now();
        if (now - opened_time_ > BOOT_GRACE && now - last_rx_time_ > RX_TIMEOUT) {
            close_serial("sem dados do Arduino ha mais de 3 s");
        }
    }




    void check_command_timeout() {
       if (stop_sent_)
        {
            return;
        }

        const auto now =
            std::chrono::steady_clock::now();

        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_cmd_time_);

        if (elapsed.count() > 250){
            RCLCPP_WARN(
                this->get_logger(),
                "Timeout de cmd_vel (%ld ms). Parando motores",
                static_cast<long>(elapsed.count())
            );

            if(send_motor_command(0, 0)){
                stop_sent_ = true;
            }
        }
    }

    int16_t linear_speed_to_pwm(double speed)
    {
        const double magnitude = std::abs(speed);

        // Comando zero = motores parados
        if (magnitude < 0.001)
        {
            return 0;
        }

        int16_t pwm = 0;

        /*
        * Calibracao experimental atual:
        *
        * 0.132 m/s -> PWM 100
        * 0.199 m/s -> PWM 150
        * 0.260 m/s -> PWM 190   (ponto suavizado)
        * 0.280 m/s -> PWM 200
        * 0.292 m/s -> PWM 215
        *
        * Abaixo de ~0.132 m/s, usamos PWM 100,
        * pois e o menor PWM operacional escolhido.
        */

        if (magnitude <= 0.132)
        {
            pwm = 100;
        }
        else if (magnitude <= 0.199)
        {
            pwm = static_cast<int16_t>(std::round(
                100.0 +
                (magnitude - 0.132) *
                (150.0 - 100.0) /
                (0.199 - 0.132)
            ));
        }
        else if (magnitude <= 0.260)
        {
            pwm = static_cast<int16_t>(std::round(
                150.0 +
                (magnitude - 0.199) *
                (190.0 - 150.0) /
                (0.260 - 0.199)
            ));
        }
        else if (magnitude <= 0.280)
        {
            pwm = static_cast<int16_t>(std::round(
                190.0 +
                (magnitude - 0.260) *
                (200.0 - 190.0) /
                (0.280 - 0.260)
            ));
        }
        else
        {
            pwm = static_cast<int16_t>(std::round(
                200.0 +
                (std::min(magnitude, 0.292) - 0.280) *
                (215.0 - 200.0) /
                (0.292 - 0.280)
            ));
        }

        return speed < 0.0 ? -pwm : pwm;
    }

    int16_t angular_speed_to_pwm(double angular)
    {
        const double magnitude = std::abs(angular);

        if (magnitude < 0.001)
        {
            return 0;
        }

        int16_t pwm = 0;

        // Dados medidos:
        // 0.557 rad/s -> PWM 170
        // 0.666 rad/s -> PWM 200
        // 0.719 rad/s -> PWM 215

        if (magnitude <= 0.557)
        {
            pwm = 170;
        }
        else if (magnitude <= 0.666)
        {
            pwm = static_cast<int16_t>(std::round(
                170.0 +
                (magnitude - 0.557) *
                (200.0 - 170.0) /
                (0.666 - 0.557)
            ));
        }
        else
        {
            pwm = static_cast<int16_t>(std::round(
                200.0 +
                (std::min(magnitude, 0.719) - 0.666) *
                (215.0 - 200.0) /
                (0.719 - 0.666)
            ));
        }

        return angular < 0.0 ? -pwm : pwm;
    }

    void cmd_vel_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        const double linear = msg->linear.x;
        const double angular = msg->angular.z;

        int16_t left_pwm = 0;
        int16_t right_pwm = 0;

        // Movimento puramente linear
        if (std::abs(angular) < 0.001)
        {
            left_pwm = linear_speed_to_pwm(linear);
            right_pwm = left_pwm;
        }

        // Rotacao no proprio eixo
        else if (std::abs(linear) < 0.001)
        {
            const int16_t rotation_pwm =
                angular_speed_to_pwm(angular);

            left_pwm = -rotation_pwm;
            right_pwm = rotation_pwm;
        }

        /// Movimento combinado: linear + angular
        else
        {
            constexpr double EFFECTIVE_TRACK_WIDTH = 0.90;

            double left_speed =
                linear - angular * (EFFECTIVE_TRACK_WIDTH / 2.0);

            double right_speed =
                linear + angular * (EFFECTIVE_TRACK_WIDTH / 2.0);

            // Faixa operacional de cada lado obtida na calibracao linear
            constexpr double MIN_SIDE_SPEED = 0.132;
            constexpr double MAX_SIDE_SPEED = 0.292;

            // Se algum lado ultrapassar nossa velocidade maxima,
            // reduzimos os dois proporcionalmente.
            // Isso preserva a curvatura desejada.
            const double fast_speed =
                std::max(
                    std::abs(left_speed),
                    std::abs(right_speed)
                );

            if (fast_speed > MAX_SIDE_SPEED)
            {
                const double scale =
                    MAX_SIDE_SPEED / fast_speed;

                left_speed *= scale;
                right_speed *= scale;
            }

            // Um lado abaixo de 0.132 m/s fica parado (ou quase) no chao, e
            // o robo pivotava sobre ele. Nesse caso sobem os dois lados na
            // mesma proporcao (mesma curvatura, lados em velocidades
            // diferentes). Se isso estourar o maximo, faz a curva mais
            // fechada possivel (lado de dentro no minimo, de fora no maximo).
            // Antes isso virava giro no lugar a 0,56 rad/s: cada correcao
            // pequena do Nav2 andando (ex.: w=0,2) virava um giro 3x maior,
            // e o robo oscilava no lugar para um lado e para o outro.
            // Giro no lugar so quando os lados teriam sentidos opostos.
            bool rotate_in_place = false;

            const double slow_speed =
                std::min(
                    std::abs(left_speed),
                    std::abs(right_speed)
                );

            if (slow_speed < MIN_SIDE_SPEED)
            {
                const bool same_direction =
                    left_speed * right_speed > 0.0;

                if (!same_direction)
                {
                    rotate_in_place = true;
                }
                else
                {
                    const double scale = MIN_SIDE_SPEED / slow_speed;
                    left_speed *= scale;
                    right_speed *= scale;

                    double &fast_side =
                        std::abs(left_speed) > std::abs(right_speed)
                            ? left_speed
                            : right_speed;

                    if (std::abs(fast_side) > MAX_SIDE_SPEED)
                    {
                        fast_side = std::copysign(MAX_SIDE_SPEED, fast_side);
                    }
                }
            }

            if (rotate_in_place)
            {
                const int16_t rotation_pwm =
                    angular_speed_to_pwm(angular);

                left_pwm = -rotation_pwm;
                right_pwm = rotation_pwm;

                RCLCPP_INFO(
                    this->get_logger(),
                    "CURVA FECHADA -> GIRO NO LUGAR | PWM L=%d R=%d",
                    static_cast<int>(left_pwm),
                    static_cast<int>(right_pwm)
                );
            }
            else
            {
                left_pwm = linear_speed_to_pwm(left_speed);
                right_pwm = linear_speed_to_pwm(right_speed);

                RCLCPP_INFO(
                    this->get_logger(),
                    "CURVA | vL=%.3f m/s vR=%.3f m/s "
                    "| PWM L=%d R=%d",
                    left_speed,
                    right_speed,
                    static_cast<int>(left_pwm),
                    static_cast<int>(right_pwm)
                );
            }
        }

        RCLCPP_INFO(
            this->get_logger(),
            "cmd_vel | linear=%.3f m/s angular=%.3f rad/s "
            "| PWM L=%d R=%d",
            linear,
            angular,
            static_cast<int>(left_pwm),
            static_cast<int>(right_pwm)
        );

        if (send_motor_command(left_pwm, right_pwm))
        {
            last_cmd_time_ =
                std::chrono::steady_clock::now();

            stop_sent_ = false;
        }
    }

    bool send_line(const std::string & line)
    {
        if (serial_port_ < 0)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Tentativa de envio com a serial fechada"
            );

            return false;
        }

        ssize_t bytes_written =
            write(
                serial_port_,
                line.c_str(),
                line.size()
            );

        if (bytes_written < 0)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Erro ao escrever na serial: %s",
                strerror(errno)
            );

            return false;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Enviado pela serial: %s",
            line.c_str()
        );

        return true;
    }

    void read_serial(){

        /*RCLCPP_INFO(
            this->get_logger(),
            "read_serial chamado"
        );*/

        if(serial_port_ < 0){
            return;
        }
        unsigned char buffer[256];
        ssize_t bytes_read = read(
            serial_port_,
            buffer,
            sizeof(buffer)
        );
         // função teste
        /*RCLCPP_INFO(
            this->get_logger(),
            "read() retornou: %ld",
            static_cast<long>(bytes_read)
        );*/

        if(bytes_read < 0){
            const int error = errno;
            if (error == EAGAIN || error == EINTR) {
                return;
            }
            if (is_disconnect_error(error)) {
                close_serial(strerror(error));
                return;
            }
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Erro ao ler serial: %s",
                strerror(error)
            );
            return;
        }

        if (bytes_read == 0){
            return;
        }
        last_rx_time_ = std::chrono::steady_clock::now();
        std::stringstream hex_data;
        for(ssize_t i=0; i< bytes_read; i++){
            hex_data << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(buffer[i]) << " ";
        }
        /*RCLCPP_INFO(
            this->get_logger(),
            "RX HEX: %s",
            hex_data.str().c_str()
        );*/

        serial_buffer_.insert(
            serial_buffer_.end(),
            buffer,
            buffer + bytes_read
        );
        
        process_serial_buffer();

        /*RCLCPP_INFO(
            this->get_logger(),
            "Recebidos %ld bytes | Buffer acumulado: %zu bytes",
            bytes_read,
            serial_buffer_.size()
        );*/
    }

    void process_serial_buffer(){
        while(true){
            if(serial_buffer_.size()<6){
                return;
            }
            if(serial_buffer_[0] != 0xA5 || serial_buffer_[1] !=0x5A){
                serial_buffer_.erase(serial_buffer_.begin());
                continue;
            }
            uint8_t version = serial_buffer_[2];
            uint8_t type = serial_buffer_[3];
            uint16_t payload_size = static_cast<uint16_t>(serial_buffer_[4])|(static_cast<uint16_t>(serial_buffer_[5]) << 8);
            size_t total_size = 2 + 4 + payload_size + 2;
           
            if(serial_buffer_.size()<total_size){
                return;
            }

            uint16_t received_crc = static_cast<uint16_t>(serial_buffer_[total_size - 2]) | (static_cast<uint16_t>(serial_buffer_[total_size - 1]) << 8);

            uint16_t calculated_crc = 0xFFFF;

            for(size_t i = 2; i <total_size - 2; ++i){
                calculated_crc = crc16_ccitt_update(
                    calculated_crc,
                    serial_buffer_[i]
                );
            }
            /* TESTE: CRC incluindo A5 5A1000000.0F
            uint16_t crc_com_magic = 0xFFFF;
            for(size_t i =0; i < total_size - 2; ++i){
                crc_com_magic = crc16_ccitt_update(
                    crc_com_magic,
                    serial_buffer_[i]
                );
            }
                // Apenas diagnóstico
            RCLCPP_WARN(
                this->get_logger(),
                "CRC TESTE | recebido=0x%04X | sem_magic=0x%04X | com_magic=0x%04X",
                static_cast<unsigned int>(received_crc),
                static_cast<unsigned int>(calculated_crc),
                static_cast<unsigned int>(crc_com_magic)
            );*/


            if(received_crc != calculated_crc){
                uint16_t scan_sequence_debug = 0;

                if(type == 0x10 && payload_size >= 2){
                    scan_sequence_debug = read_u16_le(
                        serial_buffer_.data() + 6
                    );
                }

                RCLCPP_WARN(
                    this->get_logger(),
                    "CRC invalido | type=0x%02X | seq=%u | payload=%u | recebido=0x%04X | calculado=0x%04X | xor=0x%04X",
                    static_cast<unsigned int>(type),
                    static_cast<unsigned int>(scan_sequence_debug),
                    static_cast<unsigned int>(payload_size),
                    static_cast<unsigned int>(received_crc),
                    static_cast<unsigned int>(calculated_crc),
                    static_cast<unsigned int>(received_crc ^ calculated_crc)
                );

                if(type == 0x10 && !scan_debug_printed_){
                    std::stringstream frame_hex;

                    for(size_t i = 0; i < total_size; ++i){
                        frame_hex
                            << std::hex
                            << std::setw(2)
                            << std::setfill('0')
                            << static_cast<unsigned int>(serial_buffer_[i])
                            << " ";
                    }

                    RCLCPP_WARN(
                        this->get_logger(),
                        "SCAN COMPLETO HEX: %s",
                        frame_hex.str().c_str()
                    );

                    scan_debug_printed_ = true;
                }

                serial_buffer_.erase(serial_buffer_.begin());
                continue;
            }

            // v1 = firmware mega_base_delta2d (motores esquerda/direita).
            // v2 = firmware workotvio (4 rodas + IMU BNO055).
            // A bridge responde na versao que o Arduino estiver usando.
            if(version != 0x01 && version != 0x02){
                RCLCPP_WARN(
                    this->get_logger(),
                    "Versao do protocolo invalida: %u",
                    static_cast<unsigned int>(version)
                );

                serial_buffer_.erase(serial_buffer_.begin());
                continue;
            }

            if(version != protocol_version_){
                RCLCPP_INFO(
                    this->get_logger(),
                    "Firmware usa protocolo v%u (%s)",
                    static_cast<unsigned int>(version),
                    version == 0x02 ? "4 rodas + IMU" : "esquerda/direita"
                );
                protocol_version_ = version;
            }

            RCLCPP_INFO(
                this->get_logger(),
                "Frame completo: version=%u type=0x%02X payload=%u total=%zu",
                static_cast<unsigned int>(version),
                static_cast<unsigned int>(type),
                static_cast<unsigned int>(payload_size),
                total_size
            );

            if(type == 0x10 && payload_size == 373){
                process_scan_packet(
                    serial_buffer_.data() + 6
                );
            }

            if(type == 0x11 && version == 0x01 && payload_size == 41){
                process_status_packet(
                    serial_buffer_.data() + 6
                );
            }

            if(type == 0x11 && version == 0x02 && payload_size == 88){
                process_status_packet_v2(
                    serial_buffer_.data() + 6
                );
            }

            if(type == 0x12 && payload_size == 16){
                process_imu_packet(
                    serial_buffer_.data() + 6
                );
            }

            serial_buffer_.erase(
                serial_buffer_.begin(),
                serial_buffer_.begin() + total_size
            );
        }
    }
    
    void process_scan_packet(const uint8_t *payload){
        constexpr std::size_t SCAN_BIN_COUNT = 180;
        constexpr float PI = 3.14159265358979323846F;
        constexpr float RANGE_MIN_M = 0.08F;
        constexpr float RANGE_MAX_M = 6.0F;

        uint16_t sequence = read_u16_le(payload + 0);
        uint32_t firmware_ms =read_u32_le(payload + 2);
        uint32_t scan_time_us = read_u32_le(payload + 6);
        uint8_t scan_freq_x20 = payload[10];
        uint16_t valid_bins = read_u16_le(payload + 11);

        sensor_msgs::msg::LaserScan scan;

        scan.header.frame_id = "laser";

        // O firmware agrupa as leituras em setores de 2 graus: o setor i
        // cobre [-180 + 2i, -178 + 2i). Publica cada raio no centro do setor;
        // usar o inicio girava o scan inteiro em ~1 grau.
        scan.angle_increment = (2.0F * PI) / 180.0F;
        scan.angle_min = -PI + scan.angle_increment / 2.0F;
        scan.angle_max =
            scan.angle_min +
            scan.angle_increment * 179.0F;

        scan.scan_time =
            static_cast<float>(scan_time_us) / 1000000.0F;

        // Carimbo na chegada. Carimbar no meio da volta (~90 ms antes) deixou
        // o map->odom do SLAM atrasado demais e o Nav2 passou a falhar com
        // "extrapolation into the future".
        scan.header.stamp = this->now();

        scan.time_increment =
            scan.scan_time / 180.0F;

        scan.range_min = RANGE_MIN_M;
        scan.range_max = RANGE_MAX_M;

        scan.ranges.resize(SCAN_BIN_COUNT);

        const uint8_t *ranges_data = payload + 13;

        for(std::size_t i = 0; i < SCAN_BIN_COUNT; ++i){
            uint16_t distance_mm =
                read_u16_le(ranges_data + i * 2);

            if(distance_mm == 0){
                scan.ranges[i] =
                    std::numeric_limits<float>::infinity();
            }else{
                scan.ranges[i] =
                    static_cast<float>(distance_mm) / 1000.0F;
            }
        }

        scan_pub_ ->publish(scan);

            RCLCPP_INFO(
                this->get_logger(),
                "SCAN publicado | seq=%u | firmware=%u ms | scan=%.3f s | freq_x20=%u | valid_bins=%u",
                static_cast<unsigned int>(sequence),
                static_cast<unsigned int>(firmware_ms),
                scan.scan_time,
                static_cast<unsigned int>(scan_freq_x20),
                static_cast<unsigned int>(valid_bins)
            );
    }

    void process_status_packet(const uint8_t *payload){
        uint16_t last_command_sequence = read_u16_le(payload + 0);
        uint32_t now_ms = read_u32_le(payload + 2);
        uint16_t command_age = read_u16_le(payload + 6);
        int16_t target_left_pwm = read_i16_le(payload + 8);
        int16_t target_right_pwm = read_i16_le(payload + 10);
        int16_t current_left_pwm = read_i16_le(payload + 12);
        int16_t current_right_pwm = read_i16_le(payload + 14);
        uint8_t lidar_scan_freq_x20 = payload[16];
        uint16_t last_scan_valid_bins = read_u16_le(payload + 17 );
        uint32_t lidar_good_packets = read_u32_le(payload + 19 );
        uint32_t lidar_bad_checksums = read_u32_le(payload + 23 );
        uint32_t lidar_bad_frames = read_u32_le(payload + 27 );
        uint32_t usb_rx_bad_frames = read_u32_le(payload + 31 );
        uint32_t usb_tx_dropped_frames = read_u32_le(payload + 35 );
        bool watchdog_active = payload[39] != 0;
        uint8_t lidar_motor_pwm = payload[40];

        RCLCPP_INFO(
        this->get_logger(),
        "STATUS | seq=%u | uptime=%u ms | age=%u | target L=%d R=%d | current L=%d R=%d | watchdog=%s | lidar_pwm=%u",
        static_cast<unsigned int>(last_command_sequence),
        static_cast<unsigned int>(now_ms),
        static_cast<unsigned int>(command_age),
        static_cast<int>(target_left_pwm),
        static_cast<int>(target_right_pwm),
        static_cast<int>(current_left_pwm),
        static_cast<int>(current_right_pwm),
        watchdog_active ? "ON" : "OFF",
        static_cast<unsigned int>(lidar_motor_pwm)
        );

         RCLCPP_INFO(
        this->get_logger(),
        "LiDAR | freq_x20=%u | valid_bins=%u | good=%u | bad_crc=%u | bad_frames=%u | usb_bad=%u | tx_drop=%u",
        static_cast<unsigned int>(lidar_scan_freq_x20),
        static_cast<unsigned int>(last_scan_valid_bins),
        static_cast<unsigned int>(lidar_good_packets),
        static_cast<unsigned int>(lidar_bad_checksums),
        static_cast<unsigned int>(lidar_bad_frames),
        static_cast<unsigned int>(usb_rx_bad_frames),
        static_cast<unsigned int>(usb_tx_dropped_frames)
        );
    }

    // Status do firmware v2 (88 bytes): 4 rodas, IMU e diagnostico do LiDAR.
    // Mantem as linhas "STATUS |" e "LiDAR |" no formato do v1 (L = rodas
    // esquerdas, R = direitas) para as analises de log continuarem valendo.
    void process_status_packet_v2(const uint8_t *payload){
        const uint16_t last_command_sequence = read_u16_le(payload + 0);
        const uint32_t now_ms = read_u32_le(payload + 2);
        const uint16_t command_age = read_u16_le(payload + 6);
        // ordem das rodas no firmware: FL, FR, RL, RR
        const int16_t target_fl = read_i16_le(payload + 8);
        const int16_t target_fr = read_i16_le(payload + 10);
        const int16_t current_fl = read_i16_le(payload + 16);
        const int16_t current_fr = read_i16_le(payload + 18);
        const int16_t current_rl = read_i16_le(payload + 20);
        const int16_t current_rr = read_i16_le(payload + 22);
        const uint8_t lidar_scan_freq_x20 = payload[24];
        const uint16_t last_scan_valid_bins = read_u16_le(payload + 25);
        const uint32_t lidar_good_packets = read_u32_le(payload + 27);
        const uint32_t lidar_bad_checksums = read_u32_le(payload + 31);
        const uint32_t lidar_bad_frames = read_u32_le(payload + 35);
        const uint32_t usb_rx_bad_frames = read_u32_le(payload + 39);
        const uint32_t usb_tx_dropped_frames = read_u32_le(payload + 43);
        const bool watchdog_active = payload[47] != 0;
        const uint8_t lidar_motor_pwm = payload[48];
        const bool imu_present = payload[49] != 0;
        const uint32_t max_loop_gap_us = read_u32_le(payload + 84);

        RCLCPP_INFO(
            this->get_logger(),
            "STATUS | seq=%u | uptime=%u ms | age=%u | target L=%d R=%d | current L=%d R=%d | watchdog=%s | lidar_pwm=%u | rodas FL=%d FR=%d RL=%d RR=%d | imu=%s cal=%u/%u/%u/%u",
            static_cast<unsigned int>(last_command_sequence),
            static_cast<unsigned int>(now_ms),
            static_cast<unsigned int>(command_age),
            static_cast<int>(target_fl),
            static_cast<int>(target_fr),
            static_cast<int>(current_fl),
            static_cast<int>(current_fr),
            watchdog_active ? "ON" : "OFF",
            static_cast<unsigned int>(lidar_motor_pwm),
            static_cast<int>(current_fl),
            static_cast<int>(current_fr),
            static_cast<int>(current_rl),
            static_cast<int>(current_rr),
            imu_present ? "OK" : "FALHA",
            static_cast<unsigned int>(payload[50]),
            static_cast<unsigned int>(payload[51]),
            static_cast<unsigned int>(payload[52]),
            static_cast<unsigned int>(payload[53])
        );

        RCLCPP_INFO(
            this->get_logger(),
            "LiDAR | freq_x20=%u | valid_bins=%u | good=%u | bad_crc=%u | bad_frames=%u | usb_bad=%u | tx_drop=%u | loop_max=%u us",
            static_cast<unsigned int>(lidar_scan_freq_x20),
            static_cast<unsigned int>(last_scan_valid_bins),
            static_cast<unsigned int>(lidar_good_packets),
            static_cast<unsigned int>(lidar_bad_checksums),
            static_cast<unsigned int>(lidar_bad_frames),
            static_cast<unsigned int>(usb_rx_bad_frames),
            static_cast<unsigned int>(usb_tx_dropped_frames),
            static_cast<unsigned int>(max_loop_gap_us)
        );

        if (!imu_present) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "IMU (BNO055) indisponivel no firmware"
            );
        }
    }

    // Pacote da IMU (firmware v2, 16 bytes): seq u16, ms u32,
    // heading continuo em mili-graus i32, gyro z em mrad/s i16,
    // calibracao sistema/giro/acel/mag (0-3).
    void process_imu_packet(const uint8_t *payload){
        constexpr double PI = 3.14159265358979323846;
        const uint16_t sequence = read_u16_le(payload + 0);
        const int32_t heading_mdeg = static_cast<int32_t>(read_u32_le(payload + 6));
        const int16_t gyro_z_mrad_s = read_i16_le(payload + 10);
        const uint8_t cal_system = payload[12];
        const uint8_t cal_gyro = payload[13];

        // O heading do BNO055 cresce no sentido HORARIO (bussola); no ROS o
        // yaw cresce no anti-horario. Os sinais sao parametros porque
        // dependem de como a placa foi montada: validar girando o robo.
        double yaw = static_cast<double>(heading_mdeg) * 1e-3 * PI / 180.0;
        if (imu_inverter_yaw_) {
            yaw = -yaw;
        }
        yaw = std::atan2(std::sin(yaw), std::cos(yaw));
        double yaw_rate = static_cast<double>(gyro_z_mrad_s) * 1e-3;
        if (imu_inverter_giro_) {
            yaw_rate = -yaw_rate;
        }

        sensor_msgs::msg::Imu imu;
        imu.header.stamp = this->now();
        imu.header.frame_id = imu_frame_;
        imu.orientation.z = std::sin(yaw / 2.0);
        imu.orientation.w = std::cos(yaw / 2.0);
        // So o yaw e informado; roll/pitch com variancia enorme. Variancia
        // do yaw cai conforme a calibracao do sistema (0 = pior, 3 = melhor).
        imu.orientation_covariance[0] = 1e6;
        imu.orientation_covariance[4] = 1e6;
        imu.orientation_covariance[8] = cal_system >= 3 ? 0.002 : (cal_system >= 1 ? 0.02 : 0.2);
        imu.angular_velocity.z = yaw_rate;
        imu.angular_velocity_covariance[0] = 1e6;
        imu.angular_velocity_covariance[4] = 1e6;
        imu.angular_velocity_covariance[8] = cal_gyro >= 3 ? 0.0004 : 0.004;
        // Aceleracao linear nao e enviada pelo firmware.
        imu.linear_acceleration_covariance[0] = -1.0;
        imu_pub_->publish(imu);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "IMU | seq=%u | yaw=%.1f graus | giro_z=%.3f rad/s | cal sis=%u giro=%u",
            static_cast<unsigned int>(sequence),
            yaw * 180.0 / PI,
            yaw_rate,
            static_cast<unsigned int>(cal_system),
            static_cast<unsigned int>(cal_gyro)
        );
    }

    uint16_t crc16_ccitt_update(uint16_t crc, uint8_t value){
        crc ^= static_cast<uint16_t>(value) << 8;
        for (uint8_t bit = 0; bit <8; ++bit){
            if (crc & 0x8000){
                crc = static_cast<uint16_t>((crc <<1) ^ 0x1021);
            } else{
                crc = static_cast<uint16_t>(crc << 1);
            }
        }
        return crc;
    }

    bool send_motor_command(int16_t left_pwm, int16_t right_pwm){
        if (serial_port_ < 0) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Serial desconectada: comando de motor ignorado"
            );
            return false;
        }

        std::vector<uint8_t> frame;

        frame.push_back(0xA5);
        frame.push_back(0x5A);

        frame.push_back(protocol_version_); //version
        frame.push_back(0x01); //type_motor_command

        if (protocol_version_ == 0x02) {
            // v2: uma roda por vez, na ordem FL, FR, RL, RR. As rodas sao
            // mecanum, mas o robo anda como diferencial: cada lado recebe o
            // mesmo PWM nas duas rodas.
            append_u16_le(frame, 11); //payload_size
            append_u16_le(frame, motor_sequence_);
            append_i16_le(frame, left_pwm);   // FL
            append_i16_le(frame, right_pwm);  // FR
            append_i16_le(frame, left_pwm);   // RL
            append_i16_le(frame, right_pwm);  // RR
        } else {
            append_u16_le(frame, 7); //payload_size
            append_u16_le(frame, motor_sequence_);
            append_i16_le(frame, left_pwm);
            append_i16_le(frame, right_pwm);
        }

        uint8_t flags = 0x01; //flag_enable
        frame.push_back(flags);

        uint16_t crc = 0xFFFF;

        for(size_t i = 2; i < frame.size(); ++i){
            crc = crc16_ccitt_update(crc, frame[i]);
        }
        
        append_u16_le(frame, crc);
        std::stringstream hex_data;
        for (uint8_t byte : frame){
            hex_data << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
        }
        RCLCPP_INFO(
            this->get_logger(),
            "TX HEX: %s",
            hex_data.str().c_str()
        );

        ssize_t bytes_written = write(
            serial_port_,
            frame.data(),
            frame.size()
        );

        if(bytes_written < 0){
            const int error = errno;
            if (is_disconnect_error(error)) {
                close_serial(strerror(error));
                return false;
            }
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Erro ao enviar comando de motor: %s",
                strerror(error)
            );
            return false;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Motor TX | seq=%u | left=%d | right=%d | bytes=%zu",
            static_cast<unsigned int>(motor_sequence_),
            static_cast<int>(left_pwm),
            static_cast<int>(right_pwm),
            frame.size()
        );

        ++motor_sequence_;
        return true;
    }

    void append_u16_le(std::vector<uint8_t> &data, uint16_t value){
        data.push_back(static_cast<uint8_t>(value & 0xFF));
        data.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
    }
    void append_i16_le(std::vector<uint8_t> &data, int16_t value){
        append_u16_le(
        data,
        static_cast<uint16_t>(value)
    );
    }
    
    uint16_t read_u16_le(const uint8_t *data){
        return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
    }
    int16_t read_i16_le(const uint8_t *data){
        return static_cast<int16_t>(read_u16_le(data));
    }
    uint32_t read_u32_le(const uint8_t *data){
        return static_cast<uint32_t>(data[0])|(static_cast<uint32_t>(data[1]) << 8) | (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24);
    }


    


    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    std::string imu_frame_;

    bool imu_inverter_yaw_ = true;

    bool imu_inverter_giro_ = false;

    // Versao do protocolo do firmware, detectada pelos pacotes recebidos
    // (0x01 = mega_base_delta2d, 0x02 = workotvio com IMU).
    uint8_t protocol_version_ = 0x02;

    rclcpp::TimerBase::SharedPtr serial_read_timer_;

    rclcpp::TimerBase::SharedPtr motor_test_timer_;

    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    std::chrono::steady_clock::time_point last_cmd_time_;

    rclcpp::TimerBase::SharedPtr startup_timer_;

    rclcpp::TimerBase::SharedPtr reconnect_timer_;

    std::string porta_serial_param_;

    std::string serial_path_;

    std::chrono::steady_clock::time_point last_rx_time_;

    std::chrono::steady_clock::time_point opened_time_;

    int serial_port_;

    bool stop_sent_;

    std::vector<unsigned char> serial_buffer_;

    uint16_t motor_sequence_ = 1;

    bool scan_debug_printed_ = false;
};


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node =
        std::make_shared<BridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}