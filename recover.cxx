// main.cpp

#include <vtkSmartPointer.h>
#include <vtkCubeSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkCamera.h>              // ← 추가
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkCommand.h>
#include <vtkMatrix4x4.h>
#include <vtkTransform.h>
#include <vtkInteractorStyleTrackballCamera.h>

#include <thread>
#include <mutex>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <chrono>
#include <ctime>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")
#else
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
typedef int SOCKET;
#define INVALID_SOCKET (-1)
#define SOCKET_ERROR   (-1)
#endif

// ─────────── 로깅 유틸 ───────────
static std::string currentDateTime() {
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    char buf[20];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", std::localtime(&t));
    return buf;
}
static void logInfo(const std::string& msg) {
    std::cout << "[" << currentDateTime() << "] [INFO] " << msg << "\n";
}
static void logError(const std::string& msg) {
    std::cerr << "[" << currentDateTime() << "] [ERROR] " << msg << "\n";
}

// ─────────── 전역 변수 ───────────
static std::vector<double> latest_quat = { 1.0, 0.0, 0.0, 0.0 };
static std::mutex quat_mutex;

// ─────────── 쿼터니언→행렬 ───────────
static std::array<double, 16> quat_to_matrix(double w, double x, double y, double z) {
    return { {
        1 - 2 * (y * y + z * z),  2 * (x * y - z * w),      2 * (x * z + y * w),      0,
        2 * (x * y + z * w),      1 - 2 * (x * x + z * z),  2 * (y * z - x * w),      0,
        2 * (x * z - y * w),      2 * (y * z + x * w),      1 - 2 * (x * x + y * y),  0,
        0,                  0,                  0,                  1
    } };
}

// ─────────── 네트워크 스레드 ───────────
static void network_thread(const std::string& host = "127.0.0.1", int port = 65431) {
#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        logError("WSAStartup failed");
        return;
    }
#endif

    SOCKET server = socket(AF_INET, SOCK_STREAM, 0);
    if (server == INVALID_SOCKET) {
        logError("Failed to create socket");
        return;
    }
    int opt = 1;
    setsockopt(server, SOL_SOCKET, SO_REUSEADDR, (char*)&opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    inet_pton(AF_INET, host.c_str(), &addr.sin_addr);

    if (bind(server, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
        logError("Bind failed");
        return;
    }
    listen(server, 1);
    logInfo("Waiting for connection on " + host + ":" + std::to_string(port));

    SOCKET client = accept(server, nullptr, nullptr);
    if (client == INVALID_SOCKET) {
        logError("Accept failed");
        return;
    }
    logInfo("Client connected");

    char buffer[1024];
    while (true) {
        int len = recv(client, buffer, sizeof(buffer) - 1, 0);
        if (len <= 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        buffer[len] = '\0';
        std::string text(buffer);
        if (!text.empty() && text.back() == '\n') text.pop_back();

        std::stringstream ss(text);
        std::string part;
        while (std::getline(ss, part, ';')) {
            if (part.empty()) continue;
            std::stringstream ps(part);
            std::string val;
            std::vector<double> vals;
            while (std::getline(ps, val, ',')) {
                vals.push_back(std::stod(val));
            }
            if (vals.size() == 4) {
                {
                    std::lock_guard<std::mutex> lock(quat_mutex);
                    latest_quat = vals;
                }
                logInfo(
                    "Received quaternion: w=" + std::to_string(vals[0]) +
                    ", x=" + std::to_string(vals[1]) +
                    ", y=" + std::to_string(vals[2]) +
                    ", z=" + std::to_string(vals[3])
                );
                break;
            }
        }
    }

#ifdef _WIN32
    closesocket(client);
    closesocket(server);
    WSACleanup();
#else
    close(client);
    close(server);
#endif
}

// ─────────── 키 이벤트 콜백 ───────────
class KeypressCallback : public vtkCommand {
public:
    static KeypressCallback* New() { return new KeypressCallback; }
    void Execute(vtkObject* caller, unsigned long, void*) override {
        auto iren = static_cast<vtkRenderWindowInteractor*>(caller);
        std::string key = iren->GetKeySym();
        if (key == "k" || key == "K") {
            logInfo("Key 'k' pressed: exiting.");
            iren->TerminateApp();
        }
    }
};

// ─────────── 타이머 콜백 ───────────
class TimerCallback : public vtkCommand {
public:
    vtkSmartPointer<vtkActor>       Actor;
    vtkSmartPointer<vtkRenderWindow> RenWin;
    vtkSmartPointer<vtkTransform>   Transform;
    vtkSmartPointer<vtkMatrix4x4>   B, B_inv;

    static TimerCallback* New() { return new TimerCallback; }

    TimerCallback() {
        Transform = vtkSmartPointer<vtkTransform>::New();

        B = vtkSmartPointer<vtkMatrix4x4>::New();
        B_inv = vtkSmartPointer<vtkMatrix4x4>::New();
        double Bvals[16] = {
            -1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1
        };
        B->DeepCopy(Bvals);
        vtkMatrix4x4::Invert(B, B_inv);
    }

    void Execute(vtkObject*, unsigned long, void*) override {
        std::vector<double> q;
        {
            std::lock_guard<std::mutex> lock(quat_mutex);
            q = latest_quat;
        }
        auto arr = quat_to_matrix(q[0], q[1], q[2], q[3]);
        vtkSmartPointer<vtkMatrix4x4> Mq = vtkSmartPointer<vtkMatrix4x4>::New();
        Mq->DeepCopy(arr.data());

        vtkSmartPointer<vtkMatrix4x4> tmp = vtkSmartPointer<vtkMatrix4x4>::New();
        vtkSmartPointer<vtkMatrix4x4> Mprime = vtkSmartPointer<vtkMatrix4x4>::New();
        vtkMatrix4x4::Multiply4x4(B, Mq, tmp);
        vtkMatrix4x4::Multiply4x4(tmp, B_inv, Mprime);

        Transform->SetMatrix(Mprime);
        RenWin->Render();
    }
};

int main(int, char* []) {
    // 1) 네트워크 쓰레드 시작 (람다로 감싸 기본 인자 적용)
    std::thread([]() {
        network_thread();
        }).detach();

        // 2) VTK 파이프라인 설정
        auto cube = vtkSmartPointer<vtkCubeSource>::New();
        cube->Update();

        auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(cube->GetOutputPort());

        auto actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);

        auto renderer = vtkSmartPointer<vtkRenderer>::New();
        renderer->AddActor(actor);
        renderer->SetBackground(0.1, 0.1, 0.1);

        // 카메라 Back view
        auto cam = renderer->GetActiveCamera();
        cam->SetPosition(0, 0, -5);
        cam->SetFocalPoint(0, 0, 0);
        cam->SetViewUp(0, 1, 0);
        renderer->ResetCameraClippingRange();

        auto renWin = vtkSmartPointer<vtkRenderWindow>::New();
        renWin->AddRenderer(renderer);
        renWin->SetSize(800, 600);

        auto iren = vtkSmartPointer<vtkRenderWindowInteractor>::New();
        iren->SetRenderWindow(renWin);
        iren->SetInteractorStyle(vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New());

        // 콜백 등록
        iren->AddObserver(vtkCommand::KeyPressEvent, KeypressCallback::New());

        auto timerCB = vtkSmartPointer<TimerCallback>::New();
        timerCB->Actor = actor;
        timerCB->RenWin = renWin;
        actor->SetUserTransform(timerCB->Transform);
        iren->AddObserver(vtkCommand::TimerEvent, timerCB);

        // 3) 타이머 + 렌더링
        iren->Initialize();
        iren->CreateRepeatingTimer(10);

        renWin->Render();
        iren->Start();

        logInfo("Application exited.");
        return 0;
}
