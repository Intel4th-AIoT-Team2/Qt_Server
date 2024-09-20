#include "../include/fire_robot/tab2roscontrol.h"
#include "ui_tab2roscontrol.h"
//#include "../include/fire_robot/socketclient.h"

Tab2RosControl::Tab2RosControl(int argc, char **argv, Tab1Camera *tab1Camera, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Tab2RosControl),
    m_tab1Camera(tab1Camera)
{
    ui->setupUi(this);
    prosNode = new RosNode(argc, argv);
    prosNode->pLcamView = ui->pLcamView;
    prosNode->start();

    // Server setup
    // 잘못된 IP에 연결될 경우, 세그멘테이션 오류(코어 덤프)가 발생할 수 있기에 IP지정함 -> Port 번호 변경으로 해결.
    server = new Server(this);
    if (!server->listen(QHostAddress::Any, 5001)) {   // ROS IP : 192.168.100.108
        qCritical() << "Unable to start server:" << server->errorString();
    } else {
        qDebug() << "Server started on port 5001";
    }

    connect(ui->pushButtonGo, SIGNAL(clicked()),this, SLOT(goal_Pub()));
    connect(ui->pPBFront, SIGNAL(clicked()),this, SLOT(goal_Pub_Front()));
    connect(ui->pPBLiving, SIGNAL(clicked()),this, SLOT(goal_Pub_Living()));
    connect(ui->pPBStudy, SIGNAL(clicked()),this, SLOT(goal_Pub_Study()));
    connect(ui->pPBedroom, SIGNAL(clicked()),this, SLOT(goal_Pub_Bedroom()));
    connect(ui->sendBtn, SIGNAL(clicked()),this, SLOT(sendData()));
    connect(prosNode, SIGNAL(sigLdsReceive(float *)),this, SLOT(slotLdsReceive(float *)));
    connect(prosNode, SIGNAL(rosShutdown()),qApp, SLOT(quit()));

    connect(this, SIGNAL(signalRequestRosImage(cv::Mat&)), prosNode, SLOT(slotCopyRosImage(cv::Mat&)));
    connect(server, SIGNAL(newConnect(QTcpSocket*)), this, SLOT(saveSocket(QTcpSocket*)));
//    connect(server, SIGNAL(newConnect()), this, [this](QTcpSocket *newSocket) {
//        clientSocket = newSocket; // 클라이언트 소켓 저장
//    });
}

Tab2RosControl::~Tab2RosControl()
{
    delete ui;
}

void Tab2RosControl::goal_Pub()
{
    prosNode->go_goal("map",ui->doubleSpinBox1->value(),ui->doubleSpinBox2->value(), ui->doubleSpinBox3->value(),ui->doubleSpinBox4->value());

}
void Tab2RosControl::goal_PubSlot(double x, double y, double z, double w)
{
   prosNode->go_goal("map",x,y,z,w);
}
void Tab2RosControl::goal_Pub_Front()
{
   prosNode->go_goal("map",0.83, 2.92, 0.0, 0.72);
}
void Tab2RosControl::goal_Pub_Living()
{
   prosNode->go_goal("map",0.077, 0.19, 0.0, 0.79);
}
void Tab2RosControl::goal_Pub_Study()
{
   prosNode->go_goal("map",4.96, 6.366, 0.0, 0.62);
}
void Tab2RosControl::goal_Pub_Bedroom()
{
   prosNode->go_goal("map",7.78, 1.05, 0.0, 0.59);
}
void Tab2RosControl::set_Pub()
{
   prosNode->set_goal("map",0.83, 2.92, 0.0, 0.72);
}
void Tab2RosControl::slotLdsReceive(float *pscanData)
{
    ui->lcdNumber1->display(pscanData[0]);
    ui->lcdNumber2->display(pscanData[1]);
    ui->lcdNumber3->display(pscanData[2]);
    ui->lcdNumber4->display(pscanData[3]);
}
void Tab2RosControl::sendBuzzerOff()
{
    qDebug() << "sendBuzzerOff";

    QTcpSocket* clientSocket = m_tab1Camera->getClientSocket();

    QString message = "buzzer_off";

    // 소켓이 존재하고 열려 있는지 확인
    if (!clientSocket) {
        qCritical() << "Client socket does not exist.";
        return;
    }

    if (!clientSocket->isOpen()) {
        qCritical() << "Client socket is not open.";
        return;
    }

    clientSocket->write(message.toUtf8());

    qDebug() << "Message sent to client:" << message;

    // 클라이언트의 IP 주소와 포트 번호를 가져옵니다.
    QString clientInfo = QString("IP: %1, Port: %2")
        .arg(clientSocket->peerAddress().toString())
        .arg(clientSocket->peerPort());

    qDebug() << "Message sent to client:" << message << "->" << clientInfo;
}

void Tab2RosControl::sendData(/*QString data*/)
{
    // 소켓이 존재하고 열려 있는지 확인
    if (!clientSocket) {
        qCritical() << "Client socket does not exist.";
        return;
    }

    if (!clientSocket->isOpen()) {
        qCritical() << "Client socket is not open.";
        return;
    }

    // 소켓이 정상적으로 연결되어 있는 경우
    QString message = "[18]GOAL@-1.2@-5.4";
    clientSocket->write(message.toUtf8());
    qDebug() << "Message sent to client:" << message;

    // 클라이언트의 IP 주소와 포트 번호를 가져옵니다.
    QString clientInfo = QString("IP: %1, Port: %2")
        .arg(clientSocket->peerAddress().toString())
        .arg(clientSocket->peerPort());

    qDebug() << "Message sent to client:" << message << "->" << clientInfo;
}

void Tab2RosControl::sendGoalMessage(QString msg)
{
    if (clientSocket && clientSocket->isOpen())
    {
        qDebug() << "Goal message : " << msg;
        clientSocket->write(msg.toUtf8());
        QString clientInfo = QString("IP: %1, Port: %2")
            .arg(clientSocket->peerAddress().toString())
            .arg(clientSocket->peerPort());
    }
    else
    {
        qDebug() << "Message sent to client: failed";
        qCritical() << "No client connected or socket not open!";
    }
}

void Tab2RosControl::saveSocket(QTcpSocket* socket)
{
    clientSocket = socket;
    connect(clientSocket, SIGNAL(readyRead()), this, SLOT(slotReadData()));
}

// cv::Mat png 형식으로 인코딩 -> 바이트 배열로 변환
QByteArray Tab2RosControl::matToQByteArray(const cv::Mat& image) {
    cv::Mat rgbImage;
    std::vector<uchar> buffer;
    cv::cvtColor(image, rgbImage, cv::COLOR_BGR2RGB);
    cv::imencode(".jpg", rgbImage, buffer); // JPG 형식으로 인코딩
    return QByteArray(reinterpret_cast<const char*>(buffer.data()), buffer.size());
}

void Tab2RosControl::sendImageViaTcp(QTcpSocket *socket, const cv::Mat &image) {
    if (!socket || !socket->isOpen()) {
        qCritical() << "Socket is not available or not open.";
        return;
    }

    // 이미지 인코딩
    std::vector<uchar> buffer;
    if (!cv::imencode(".png", image, buffer)) {
        qCritical() << "Failed to encode image.";
        return;
    }

    // 이미지 크기와 데이터를 준비
    quint32 imageSize = buffer.size();
    QByteArray sizeArray;
    sizeArray.setNum(imageSize);

    // 이미지 크기 및 이미지 데이터 전송
    socket->write(sizeArray);
    socket->write(reinterpret_cast<const char*>(buffer.data()), buffer.size());
    socket->flush();

    qDebug() << "Image sent with size:" << imageSize << "bytes.";
}

void Tab2RosControl::sendImageInChunks(QTcpSocket *socket, const cv::Mat &image) {
    if (!socket || !socket->isOpen()) {
        qCritical() << "Socket is not available or not open.";
        return;
    }

    // 1. 이미지 인코딩
    std::vector<uchar> buffer;
    if (!cv::imencode(".png", image, buffer)) {
        qCritical() << "Failed to encode image.";
        return;
    }

    // 2. 청크 크기를 계산합니다.
    quint32 totalSize = buffer.size();
    int chunkCount = 3;
    quint32 chunkSize = std::ceil(static_cast<float>(totalSize) / chunkCount);

    // 3. 각 청크로 데이터를 나누어 보냅니다.
    for (int i = 0; i < chunkCount; ++i) {
        quint32 startIdx = i * chunkSize;
        quint32 endIdx = std::min(startIdx + chunkSize, totalSize);
        QByteArray chunkData = QByteArray::fromRawData(reinterpret_cast<const char*>(buffer.data() + startIdx), endIdx - startIdx);

        socket->write(chunkData);
        socket->flush(); // 적절히 전송 완료를 보장하기 위해 flush 사용

        qDebug() << "Chunk" << i + 1 << "sent, size:" << chunkData.size() << "bytes.";
    }

    qDebug() << "Total image sent with size:" << totalSize << "bytes.";
}

void Tab2RosControl::slotReadData() {
    if (!clientSocket) {
        qCritical() << "Client socket is not available.";
        return;
    }

    // 데이터 읽기
    QByteArray data = clientSocket->readAll();
    qDebug() << "Data received from client:" << data;
    if (data.trimmed() == "extinguish finished")
    {
        sendBuzzerOff();
    }

    if (data.trimmed() == "IMG")
    {
        // 해당 시그널 발생 시 -> rosImg에 이미지 1 프레임 복사함
        emit signalRequestRosImage(rosImg);

        // 소켓이 존재하고 열려 있는지 확인
        if (!clientSocket) {
            qCritical() << "Client socket does not exist.";
            return;
        }

        if (!clientSocket->isOpen()) {
            qCritical() << "Client socket is not open.";
            return;
        }

        QByteArray imgData = matToQByteArray(rosImg);

        // 이미지 바이너리 전송
        clientSocket->write(imgData);
        clientSocket->flush();

        qDebug() << "Image sent with size:" << imgData.size() << "bytes.";
        qDebug() << "Image sent to client.";

        // 데이터 전송 완료 후 소켓 연결 해제
        clientSocket->disconnectFromHost();

        if (clientSocket->state() == QAbstractSocket::UnconnectedState || clientSocket->waitForDisconnected(3000)) {
            qDebug() << "Socket successfully disconnected.";
        } else {
            qCritical() << "Socket disconnect failed.";
        }
    }
}

