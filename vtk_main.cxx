#define _CRT_SECURE_NO_WARNINGS

/* ===============================  INCLUDES  =============================== */
#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkNamedColors.h>
#include <vtkSphereSource.h>
#include <vtkCylinderSource.h>
#include <vtkCubeSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkPlaneSource.h>
#include <vtkTransform.h>
#include <vtkMatrix4x4.h>
#include <vtkProperty.h>
#include <vtkCommand.h>
#include <filesystem>

#include <vtkImageCanvasSource2D.h>
#include <vtkTexture.h>
#include <vtkTransformTextureCoords.h>

#include <array>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <string>
#include <cstdio>
#include <cstring>
#include <algorithm> // std::clamp


// GLM (header‑only) for vec3 ----------------------------------------------
#include <glm/glm.hpp>

// === floor clamp config ===
constexpr double kFloorY = -49.0;  // 바닥 높이(기존 floorY와 통일)
constexpr double kClampTol = 1e-3;   // 수치 떨림 방지용 아주 작은 여유

std::atomic<double> gLeftFootBottomY{ kFloorY };
std::atomic<double> gRightFootBottomY{ kFloorY };

std::atomic<double> gLeftFootMidX{ 0 }, gLeftFootMidZ{ 0 };
std::atomic<double> gRightFootMidX{ 0 }, gRightFootMidZ{ 0 };

constexpr double kContactYTol = 0.8;   // 바닥과의 Y 거리 허용치
constexpr double kLiftYExit = 1.5;   // 접지 해제(리프트) 문턱
constexpr double kVelTolPerFrame = 0.06;  // 프레임당 수직 이동 허용치(10ms 타이머 기준)
constexpr double kDeadXY = 0.30;  // XY 데드존(사소한 미끄러짐 무시)
constexpr double kLockGain = 0.60;  // 보정 게인(0~1)
constexpr double kMaxStepXY = 1.20;  // 프레임당 최대 보정 스텝(과도 스냅 방지)

// === v=0(속도-락) 보정 파라미터 ===
constexpr int kTimerIntervalMs = 10;
constexpr double kDt = kTimerIntervalMs * 0.001; // 컴파일 타임 계산됨
constexpr double kVelLpfAlpha = 0.25;   // 속도 저역통과(0~1) 클수록 민감
constexpr double kVelCancelGain = 0.40;   // v=0 수렴 게인(0~1)
constexpr double kMaxVelCancelStep = 1.8;    // 프레임당 최대 보정 스텝( VTK 단위 )


// 접지 디바운스/히스테리시스 & 램프
constexpr int    kTouchHoldFrames = 3;   // 접지로 바꾸려면 N프레임 연속 touching이어야 함
constexpr int    kReleaseHoldFrames = 2;   // 접지 해제도 N프레임 연속 떨어져 있어야 함
constexpr int    kLockRampFrames = 6;   // 접지 직후 풋락 게인 램프-업 기간(프레임 단위)
constexpr float kAnchorAlpha = 0.35;// 앵커 스무딩(0=이전 유지, 1=즉시 스냅)

// 수직 클램프 1프레임 최대 보정(팝 방지)
constexpr double kMaxYClampStep = 0.8; // VTK Y 단위

// ★ 추가: 현재 발바닥의 월드 XZ 캐시(보정에 사용)
std::atomic<double> gLeftFootX{ 0.0 }, gLeftFootZ{ 0.0 };
std::atomic<double> gRightFootX{ 0.0 }, gRightFootZ{ 0.0 };

// 발바닥 월드 XZ 이전값(속도 추정용)
std::atomic<double> gPrevLeftX{0.0},  gPrevLeftZ{0.0};
std::atomic<double> gPrevRightX{0.0}, gPrevRightZ{0.0};
std::atomic_bool    gPrevLeftValid{false}, gPrevRightValid{false};

// 저역통과된 평면 속도( VTK 단위: X,Z / 초 )
std::atomic<double> gFiltLeftVX{0.0},  gFiltLeftVZ{0.0};
std::atomic<double> gFiltRightVX{0.0}, gFiltRightVZ{0.0};


namespace fs = std::filesystem;

/* ===============================  WINSOCK  =============================== */
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")

/* ===============================  CONFIG  ================================ */
constexpr const char* kHost = "127.0.0.1";
constexpr uint16_t     kPort = 65431;
constexpr int          kSensorCount = 7;      // 7 IMUs
//constexpr int          kTimerIntervalMs = 10; // VTK timer period (10 ms)

/* ===============================  TYPES  ================================= */
struct Quaternion { double w{ 1 }, x{ 0 }, y{ 0 }, z{ 0 }; };

// 스탠스 앵커 상태
struct PlantState {
    bool planted{ false };
    glm::vec3 anchorPos{ 0.0f, (float)kFloorY, 0.0f };
    double anchorYaw{ 0.0 };
    double prevBottomY{ kFloorY };

    // ★ 추가: 팝 방지용 상태
    int  touchHold{ 0 };       // 접지 후보 연속 프레임 수
    int  releaseHold{ 0 };     // 해제 후보 연속 프레임 수
    int  lockAge{ 0 };         // 접지 이후 경과 프레임(풋락 램프-업)
    bool everPlanted{ false }; // 첫 접지 이후인지(앵커 스무딩 초기화용)
};
PlantState gPlantL, gPlantR;

enum class SensorIndex : int {
    PELVIS = 0, LEFT_LEG, LEFT_CALF, LEFT_FOOT,
    RIGHT_LEG, RIGHT_CALF, RIGHT_FOOT,
    COUNT
};

enum class Foot { NONE, LEFT, RIGHT };
std::atomic<Foot> gActiveStance{ Foot::NONE };
glm::vec3 gLastAnchorWorld = glm::vec3(0.0f, (float)kFloorY, 0.0f);

// VTK 좌표계 기준 최소 스텝 길이(노이즈 무시용)
// (X,Y,Z 모두 VTK 단위. 현재 코드에서 Sensor X/Y는 VTK X/Z로 5배 스케일)
constexpr double kStepMinWorld = 0.30;    // 필요시 0.2~0.5 사이 튜닝

/* =====================  GLOBAL STATE & SYNCHRO  =========================== */
std::array<Quaternion, kSensorCount> gLatestQuat{};         // live quats
std::array<glm::vec3, kSensorCount> gLatestPos{};          // live positions ★★★
std::array<Quaternion, kSensorCount> gCalibrationOffset{};  // per‑sensor offset
std::array<std::array<double, 16>, kSensorCount> gBaseTransforms; // B matrices

std::mutex               gMutex;
std::atomic_bool         gSaving{ false };
std::vector<std::vector<double>> gSensorLog;  // ts + 7×7 floats

glm::vec3 gPosOffset = glm::vec3(0.0f);

/* ======================  QUATERNION MATH HELPERS  ======================== */
static inline Quaternion quatInverse(const Quaternion& q) {
    return { q.w, -q.x, -q.y, -q.z };
}
static inline Quaternion quatMultiply(const Quaternion& a, const Quaternion& b) {
    return {
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
    };
}
static inline std::array<double, 16> quatToMatrix(const Quaternion& q) {
    double w = q.w, x = q.x, y = q.y, z = q.z;
    return {
        1 - 2 * (y * y + z * z),  2 * (x * y - z * w),  2 * (x * z + y * w),  0,
        2 * (x * y + z * w),  1 - 2 * (x * x + z * z),  2 * (y * z - x * w),  0,
        2 * (x * z - y * w),  2 * (y * z + x * w),  1 - 2 * (x * x + y * y),  0,
        0,0,0,1
    };
}

/* ======================  VTK SOURCE FACTORIES  =========================== */
static vtkSmartPointer<vtkCubeSource> makeCube(double x, double y, double z) {
    auto src = vtkSmartPointer<vtkCubeSource>::New();
    src->SetXLength(x); src->SetYLength(y); src->SetZLength(z); src->SetCenter(0, 0, -2);
    return src;
}
static vtkSmartPointer<vtkSphereSource> makeSphere(double r) {
    auto s = vtkSmartPointer<vtkSphereSource>::New();
    s->SetRadius(r); s->SetPhiResolution(100); s->SetThetaResolution(100);
    return s;
}
static vtkSmartPointer<vtkCylinderSource> makeCylinder(double r, double h) {
    auto c = vtkSmartPointer<vtkCylinderSource>::New();
    c->SetRadius(r); c->SetHeight(h); c->SetResolution(100); return c;
}
static vtkSmartPointer<vtkActor> makeActor(vtkSmartPointer<vtkPolyDataAlgorithm> src,
    vtkSmartPointer<vtkTransform> tr,
    vtkSmartPointer<vtkNamedColors> colors,
    const std::string& colorName = "Cornsilk") {
    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(src->GetOutputPort());
    auto act = vtkSmartPointer<vtkActor>::New();
    act->SetMapper(mapper);
    act->GetProperty()->SetColor(colors->GetColor3d(colorName).GetData());
    act->SetUserTransform(tr);
    return act;
}

/* ======================  FILE / TIME HELPERS  ============================ */
static std::string timestampedFilename() {
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{}; localtime_s(&tm, &t);
    char buf[64]; std::strftime(buf, sizeof(buf), "./Recorded_Data/sensor_data_%Y%m%d_%H%M%S.csv", &tm);
    fs::create_directories("./Recorded_Data");
    return buf;
}

/* ======================  TCP READER THREAD  ============================== */
static void networkThread() {
    WSADATA wsa; if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) { printf("[NET] WSAStartup fail\n"); return; }
    SOCKET sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock == INVALID_SOCKET) { printf("[NET] socket() fail: %d\n", WSAGetLastError()); return; }

    sockaddr_in addr{}; addr.sin_family = AF_INET; addr.sin_port = htons(kPort);
    inet_pton(AF_INET, kHost, &addr.sin_addr);

    if (bind(sock, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
        printf("[NET] bind() fail: %d (port in use?)\n", WSAGetLastError());
        closesocket(sock); WSACleanup(); return;
    }
    listen(sock, 1);
    printf("[NET] listening on %s:%d\n", kHost, kPort);

    SOCKET client = accept(sock, nullptr, nullptr);
    if (client == INVALID_SOCKET) {
        printf("[NET] accept() fail: %d\n", WSAGetLastError());
        closesocket(sock); WSACleanup(); return;
    }
    printf("[NET] client connected\n");

    const size_t kPktSize = 7 * sizeof(float);
    const size_t kFrameSize = kPktSize * kSensorCount;
    std::vector<char> buf(kFrameSize);

    size_t frameCount = 0;
    while (true) {
        int recvd = 0;
        while (recvd < (int)kFrameSize) {
            int n = recv(client, buf.data() + recvd, (int)kFrameSize - recvd, 0);
            if (n <= 0) { printf("[NET] recv end: n=%d, err=%d\n", n, WSAGetLastError()); goto cleanup; }
            recvd += n;
        }
        frameCount++;
        if ((frameCount % 10000) == 0) printf("[NET] frames: %zu\n", frameCount);

        const float* f = reinterpret_cast<const float*>(buf.data());
        {
            std::lock_guard lk(gMutex);
            for (int i = 0; i < kSensorCount; ++i, f += 7) {
                gLatestQuat[i] = { f[0], f[1], f[2], f[3] };
                gLatestPos[i] = { f[4], f[5], f[6] };
            }
        }
    }
cleanup:
    closesocket(client); closesocket(sock); WSACleanup();
    printf("[NET] cleanup\n");
}
static void startNetwork() { std::thread(networkThread).detach(); }

/* ======================  KEYBOARD CALLBACK  ============================== */
class KeyPressCallback : public vtkCommand {
public:
    static KeyPressCallback* New() { return new KeyPressCallback; }
    void Execute(vtkObject* caller, unsigned long, void*) override {
        auto iren = static_cast<vtkRenderWindowInteractor*>(caller);
        std::string key = iren->GetKeySym();
        if (key == "r") { gSaving = true; gSensorLog.clear(); printf("[LOG] recording...\n"); }
        else if (key == "s") {
            gSaving = false; printf("[LOG] stop & save\n");
            std::ofstream csv(timestampedFilename());
            // header
            csv << "timestamp";
            for (int i = 1; i <= kSensorCount; ++i) {
                csv << ",sensor" << i << "_w, sensor" << i << "_x, sensor" << i << "_y, sensor" << i << "_z";
                csv << ", sensor" << i << "_px, sensor" << i << "_py, sensor" << i << "_pz";           // ★★★
            }
            csv << "\n";
            // rows
            for (auto& row : gSensorLog) {
                for (size_t i = 0; i < row.size(); ++i) csv << (i ? "," : "") << row[i];
                csv << "\n";
            }
            printf("[LOG] CSV saved (%zu rows)\n", gSensorLog.size());
        }
        else if (key == "c") { std::lock_guard lk(gMutex); gCalibrationOffset = gLatestQuat; printf("[LOG] calibrated\n"); }
        else if (key == "o") {
            std::lock_guard lk(gMutex);
            const auto& p0 = gLatestPos[(int)SensorIndex::PELVIS];
            gPosOffset = glm::vec3(-p0.x, -p0.y, -p0.z);
            printf("[LOG] origin reset: offset=(%.3f, %.3f, %.3f)\n", gPosOffset.x, gPosOffset.y, gPosOffset.z);
        }
    }
};

/* ======================  TIMER CALLBACK  ================================ */
class SensorTimerCallback : public vtkCommand {
public:
    SensorTimerCallback(vtkActor* actor, vtkTransform* tr, double posX, double posY, int sensorIdx,
        vtkRenderWindow* win, vtkTransform* parentTr = nullptr)
        : mActor(actor), mTransform(tr), mPosX(posX), mPosY(posY), mSensorIdx(sensorIdx),
        mRenWin(win), mParent(parentTr) {
    }

    static SensorTimerCallback* New(vtkActor* a = nullptr, vtkTransform* t = nullptr,
        double x = 0, double y = 0, int idx = 0,
        vtkRenderWindow* w = nullptr, vtkTransform* p = nullptr) {
        return new SensorTimerCallback(a, t, x, y, idx, w, p);
    }

    void Execute(vtkObject*, unsigned long, void*) override {
        Quaternion raw, calib;
        glm::vec3 pos;

        {
            std::lock_guard lk(gMutex);
            raw = gLatestQuat[mSensorIdx];
            calib = gCalibrationOffset[mSensorIdx];
            pos = gLatestPos[mSensorIdx];
        }

        Quaternion adj = quatMultiply(raw, quatInverse(calib));

        // 부모 관절 기준 상대 회전 계산
        if (mParent) {
            int parentIdx = getParentSensorIndex(mSensorIdx);
            if (parentIdx >= 0) {
                Quaternion parentRaw, parentCalib;
                {
                    std::lock_guard lk(gMutex);
                    parentRaw = gLatestQuat[parentIdx];
                    parentCalib = gCalibrationOffset[parentIdx];
                }
                Quaternion parentAdj = quatMultiply(parentRaw, quatInverse(parentCalib));
                adj = quatMultiply(quatInverse(parentAdj), adj);
            }
        }

        auto Mq = quatToMatrix({ adj.w, adj.y, adj.z, adj.x });
        //auto Mq = quatToMatrix({ adj.w, adj.x, adj.y, adj.z });
        vtkNew<vtkMatrix4x4> Q;
        Q->DeepCopy(Mq.data());

        mTransform->Identity();

        // ★★★ 핵심 수정: 회전 축(기준점) 수정 ★★★
        if (mSensorIdx == static_cast<int>(SensorIndex::PELVIS)) {
            // PELVIS: 절대 위치 + 중심 회전
            glm::vec3 offset;
            constexpr double kMetersToVtk = 5.0;

            { std::lock_guard lk(gMutex); offset = gPosOffset; }
            auto p = pos + offset;
            // mTransform->Translate(p.x, p.y, p.z);
            // mTransform->Concatenate(Q);
            mTransform->Translate(p.x * kMetersToVtk, p.z, p.y * kMetersToVtk); //★★★
            mTransform->Concatenate(Q);
        }
        
        else {
            // ★★★ 관절들: 연결점에서 회전하도록 수정 ★★★

            // 1. 관절 연결점으로 이동 (부모와의 연결점)
            double jointX = 0, jointY = getJointOffsetY(mSensorIdx);  // 연결점 Y 좌표
            mTransform->Translate(jointX, jointY, 0);

            // 2. 연결점에서 회전
            mTransform->Concatenate(Q);

            // 3. 관절 중심으로 오프셋 (기하학적 중심까지의 거리)
            double centerOffsetY = getCenterOffsetY(mSensorIdx);
            mTransform->Translate(0, centerOffsetY, 0);
        }

        // 로깅 (센서 0에서만 한 번)
        if (mSensorIdx == 0 && gSaving.load()) {
            double ts = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
            std::vector<double> row;
            row.reserve(1 + 7 * kSensorCount);
            row.push_back(ts);

            std::lock_guard lk(gMutex);
            for (int i = 0; i < kSensorCount; ++i) {
                const auto& q = gLatestQuat[i];
                const auto& p = gLatestPos[i];
                row.push_back(q.w); row.push_back(q.x); row.push_back(q.y); row.push_back(q.z);
                row.push_back(p.x); row.push_back(p.y); row.push_back(p.z);
            }
            gSensorLog.emplace_back(std::move(row));
        }


        // ----- (A) 모든 발 프레임: 자신의 최저 Y(=bottomY)와 월드 XZ 저장 -----
        if (mSensorIdx == static_cast<int>(SensorIndex::LEFT_FOOT) ||
            mSensorIdx == static_cast<int>(SensorIndex::RIGHT_FOOT))
        {
            // 1) 발 액터의 월드 바운딩 박스 → 최저 Y(=bottomY)
            double b[6];
            mActor->GetBounds(b);     // [xmin,xmax, ymin,ymax, zmin,zmax]
            double bottomY = b[2];

            // 2) 발바닥 "중심" 월드 좌표를 정확히 구함
            //    로컬에서 발 큐브의 중심은 (0,0,0), 높이 3 → 바닥은 y=-1.5
            double localBottom[3] = { 0.0, -1.5, 0.0 };
            double worldBottom[3];
            mTransform->TransformPoint(localBottom, worldBottom); // 입력/부모 변환 포함

            // 3) 전역 캐시 갱신
            if (mSensorIdx == static_cast<int>(SensorIndex::LEFT_FOOT)) {
                gLeftFootBottomY.store(bottomY, std::memory_order_relaxed);
                gLeftFootX.store(worldBottom[0], std::memory_order_relaxed);
                gLeftFootZ.store(worldBottom[2], std::memory_order_relaxed);
            }
            else {
                gRightFootBottomY.store(bottomY, std::memory_order_relaxed);
                gRightFootX.store(worldBottom[0], std::memory_order_relaxed);
                gRightFootZ.store(worldBottom[2], std::memory_order_relaxed);
            }

            const double curX = worldBottom[0]; // VTK X
            const double curZ = worldBottom[2]; // VTK Z

            if (mSensorIdx == static_cast<int>(SensorIndex::LEFT_FOOT)) {
                if (!gPrevLeftValid.load(std::memory_order_relaxed)) {
                    gPrevLeftX.store(curX, std::memory_order_relaxed);
                    gPrevLeftZ.store(curZ, std::memory_order_relaxed);
                    gPrevLeftValid.store(true, std::memory_order_relaxed);
                }
                else {
                    const double px = gPrevLeftX.load(std::memory_order_relaxed);
                    const double pz = gPrevLeftZ.load(std::memory_order_relaxed);
                    const double vx = (curX - px) / kDt;   // VTK/s
                    const double vz = (curZ - pz) / kDt;
                    gPrevLeftX.store(curX, std::memory_order_relaxed);
                    gPrevLeftZ.store(curZ, std::memory_order_relaxed);

                    // 1차 IIR LPF
                    const double fx = (1.0 - kVelLpfAlpha) * gFiltLeftVX.load(std::memory_order_relaxed)
                        + kVelLpfAlpha * vx;
                    const double fz = (1.0 - kVelLpfAlpha) * gFiltLeftVZ.load(std::memory_order_relaxed)
                        + kVelLpfAlpha * vz;
                    gFiltLeftVX.store(fx, std::memory_order_relaxed);
                    gFiltLeftVZ.store(fz, std::memory_order_relaxed);
                }
            }
            else { // RIGHT_FOOT
                if (!gPrevRightValid.load(std::memory_order_relaxed)) {
                    gPrevRightX.store(curX, std::memory_order_relaxed);
                    gPrevRightZ.store(curZ, std::memory_order_relaxed);
                    gPrevRightValid.store(true, std::memory_order_relaxed);
                }
                else {
                    const double px = gPrevRightX.load(std::memory_order_relaxed);
                    const double pz = gPrevRightZ.load(std::memory_order_relaxed);
                    const double vx = (curX - px) / kDt;   // VTK/s
                    const double vz = (curZ - pz) / kDt;
                    gPrevRightX.store(curX, std::memory_order_relaxed);
                    gPrevRightZ.store(curZ, std::memory_order_relaxed);

                    // 1차 IIR LPF
                    const double fx = (1.0 - kVelLpfAlpha) * gFiltRightVX.load(std::memory_order_relaxed)
                        + kVelLpfAlpha * vx;
                    const double fz = (1.0 - kVelLpfAlpha) * gFiltRightVZ.load(std::memory_order_relaxed)
                        + kVelLpfAlpha * vz;
                    gFiltRightVX.store(fx, std::memory_order_relaxed);
                    gFiltRightVZ.store(fz, std::memory_order_relaxed);
                }
            }

            // 4) 접지(스탠스) 판정 및 앵커 업데이트
            PlantState& ps = (mSensorIdx == static_cast<int>(SensorIndex::LEFT_FOOT)) ? gPlantL : gPlantR;

            double vY = bottomY - ps.prevBottomY;      // 프레임당 수직 위치 변화량(속도 근사)
            ps.prevBottomY = bottomY;

            bool nearFloor = std::abs(bottomY - kFloorY) <= kContactYTol;
            bool verticalCalm = std::abs(vY) <= kVelTolPerFrame;
            bool touching = (nearFloor && verticalCalm);

            // ---- 디바운스 카운터 업데이트 ----
            if (touching) { ps.touchHold++; ps.releaseHold = 0; }
            else { ps.releaseHold++; ps.touchHold = 0; }

            // ---- 스탠스 진입 ----
            if (!ps.planted && ps.touchHold >= kTouchHoldFrames) {
                ps.planted = true;
                ps.anchorPos = glm::vec3((float)worldBottom[0], (float)kFloorY, (float)worldBottom[2]);
                ps.lockAge = 0;
                bool wasEver = ps.everPlanted;
                ps.everPlanted = true;

                // 새 앵커
                //glm::vec3 newAnchor((float)worldBottom[0], (float)kFloorY, (float)worldBottom[2]);


                // ★ 여기서만: 스텝 오도메트리 누적 + 양발 앵커 co-shift
                    // --- 스텝 오도메트리 누적 + 앵커 co-shift ---
                glm::vec3 newAnchor((float)worldBottom[0], (float)kFloorY, (float)worldBottom[2]);

                Foot thisFoot = (mSensorIdx == (int)SensorIndex::LEFT_FOOT) ? Foot::LEFT : Foot::RIGHT;
                Foot prevFoot = gActiveStance.load(std::memory_order_relaxed);

                if (prevFoot == Foot::NONE) {
                    gLastAnchorWorld = newAnchor;
                    gActiveStance.store(thisFoot, std::memory_order_relaxed);
                }
                else if (prevFoot != thisFoot) {
                    glm::vec3 delta = newAnchor - gLastAnchorWorld;            // VTK 평면 Δ(X,Z)
                    double planar = std::hypot((double)delta.x, (double)delta.z);
                    if (planar > kStepMinWorld) {
                        constexpr double kMetersToVtk = 5.0;
                        std::lock_guard lk(gMutex);

                        // 전역 오프셋 누적
                        gPosOffset.x += (float)(delta.x / kMetersToVtk); // 좌우(X)
                        gPosOffset.y += (float)(delta.z / kMetersToVtk); // 앞뒤(Y)  ← 중요!

                        // 앵커들도 같은 Δ만큼 같이 이동(co-shift) → 풋락이 되돌리지 않게
                        gPlantL.anchorPos.x += delta.x; gPlantL.anchorPos.z += delta.z;
                        gPlantR.anchorPos.x += delta.x; gPlantR.anchorPos.z += delta.z;

                        
                    }
                    gLastAnchorWorld = newAnchor;
                    gActiveStance.store(thisFoot, std::memory_order_relaxed);

                    printf("[ODOM] dX=%.3f dZ=%.3f | gPos=(%.3f,%.3f,%.3f)\n",
                        delta.x, delta.z, gPosOffset.x, gPosOffset.y, gPosOffset.z);
                }

            }

            // ---- 스탠스 유지/해제 ----
            if (ps.planted) {
                // 램프-업(풋락 게인 용)
                if (ps.lockAge < kLockRampFrames) ps.lockAge++;

                // 충분히 떨어진 상태가 일정 프레임 지속되면 해제
                bool liftedEnough = (bottomY > kFloorY + kLiftYExit);
                if (liftedEnough && ps.releaseHold >= kReleaseHoldFrames) {
                    ps.planted = false;
                    ps.lockAge = 0;
                }
            }

            // 스탠스 이탈(점프/스윙 시작): 충분히 바닥에서 떨어지면 해제
            if (ps.planted && (bottomY > kFloorY + kLiftYExit)) {
                ps.planted = false;
            }
        }

        // ----- (B) 프레임당 1번: RIGHT_FOOT에서만 "센서 Z(수직) 클램프 + 센서 XY 평면 풋락" 수행 -----
        if (mSensorIdx == static_cast<int>(SensorIndex::RIGHT_FOOT)) {

            // 1) 수직(Z_sens=VTK Y) 클램프
            const double leftB = gLeftFootBottomY.load(std::memory_order_relaxed);
            const double rightB = gRightFootBottomY.load(std::memory_order_relaxed);
            const double minBottom = std::min(leftB, rightB);

            //double deltaY_vtk = 0.0; // VTK Y == Sensor Z
            //if (deltaY_vtk != 0.0) {
            //    // ★ 1프레임 최대 보정 제한
            //    deltaY_vtk = std::clamp(deltaY_vtk, -kMaxYClampStep, kMaxYClampStep);

            //    std::lock_guard lk(gMutex);
            //    gPosOffset.z += static_cast<float>(deltaY_vtk);
            //}
            //if (minBottom < kFloorY - kClampTol) deltaY_vtk = kFloorY - minBottom; // 뚫림 → 위로
            //else if (minBottom > kFloorY + kClampTol) deltaY_vtk = kFloorY - minBottom; // 부양 → 아래로

            //if (deltaY_vtk != 0.0) {
            //    std::lock_guard lk(gMutex);
            //    // 주의: VTK Y는 gPosOffset.z에 매핑됨 (pelvis Translate에서 확인)
            //    gPosOffset.z += static_cast<float>(deltaY_vtk);
            //}
            double deltaY_vtk = 0.0;
            if (minBottom < kFloorY - kClampTol)      deltaY_vtk = kFloorY - minBottom;
            else if (minBottom > kFloorY + kClampTol) deltaY_vtk = kFloorY - minBottom;

            deltaY_vtk = std::clamp(deltaY_vtk, -kMaxYClampStep, kMaxYClampStep);
            if (deltaY_vtk != 0.0) {
                std::lock_guard lk(gMutex);
                gPosOffset.z += static_cast<float>(deltaY_vtk); // VTK Y ↔ gPosOffset.z
            }

            // 2) 평면 풋락: 센서 XY 고정  (센서 XY == VTK XZ)
            const bool L = gPlantL.planted;
            const bool R = gPlantR.planted;



            if (L || R) {
                // 평균 slip 속도( VTK/s )
                double vX = 0.0, vZ = 0.0; int cnt = 0;
                if (L) {
                    vX += gFiltLeftVX.load(std::memory_order_relaxed);
                    vZ += gFiltLeftVZ.load(std::memory_order_relaxed); ++cnt;
                }
                if (R) {
                    vX += gFiltRightVX.load(std::memory_order_relaxed);
                    vZ += gFiltRightVZ.load(std::memory_order_relaxed); ++cnt;
                }
                if (cnt > 0) { vX /= cnt; vZ /= cnt; }

                // v=0 유도: Δoffset( VTK/frame ) = -gain * v * dt
                double stepX_vtk = std::clamp(-kVelCancelGain * vX * kDt,
                    -kMaxVelCancelStep, kMaxVelCancelStep);
                double stepZ_vtk = std::clamp(-kVelCancelGain * vZ * kDt,
                    -kMaxVelCancelStep, kMaxVelCancelStep);

                if (std::abs(stepX_vtk) > 0.0 || std::abs(stepZ_vtk) > 0.0) {
                    constexpr double kMetersToVtk = 5.0; // 파일 내 기존 스케일 상수와 동일
                    std::lock_guard lk(gMutex);
                    // VTK X ← gPosOffset.x,    VTK Z ← gPosOffset.y  (PELVIS Translate 참고)
                    gPosOffset.x += static_cast<float>(stepX_vtk / kMetersToVtk);
                    gPosOffset.y += static_cast<float>(stepZ_vtk / kMetersToVtk);
                }
            }



            if (L || R) {


                // 현재 발바닥 위치 (VTK XZ ← Sensor XY)
                const double curLX_vtkX = gLeftFootX.load(std::memory_order_relaxed);
                const double curLY_sens = gLeftFootZ.load(std::memory_order_relaxed);   // VTK Z == Sensor Y

                const double curRX_vtkX = gRightFootX.load(std::memory_order_relaxed);
                const double curRY_sens = gRightFootZ.load(std::memory_order_relaxed);  // VTK Z == Sensor Y

                // 앵커 (VTK XZ ← Sensor XY)
                const double tgtLX_vtkX = gPlantL.anchorPos.x;
                const double tgtLY_sens = gPlantL.anchorPos.z;  // 저장 시 z필드는 VTK Z(=Sensor Y)

                const double tgtRX_vtkX = gPlantR.anchorPos.x;
                const double tgtRY_sens = gPlantR.anchorPos.z;

                // 에러(양발 접지면 평균)
                double eX_sens = 0.0, eY_sens = 0.0;
                int cnt = 0;
                if (L) { eX_sens += (tgtLX_vtkX - curLX_vtkX); eY_sens += (tgtLY_sens - curLY_sens); ++cnt; }
                if (R) { eX_sens += (tgtRX_vtkX - curRX_vtkX); eY_sens += (tgtRY_sens - curRY_sens); ++cnt; }
                if (cnt > 0) { eX_sens /= cnt; eY_sens /= cnt; }

                // 데드존
                if (std::abs(eX_sens) < kDeadXY) eX_sens = 0.0;
                if (std::abs(eY_sens) < kDeadXY) eY_sens = 0.0;

                if (eX_sens != 0.0 || eY_sens != 0.0) {
                    // ★ 동적 게인: 접지 경과 프레임(lockAge)로 램프-업(0~1)
                    double rampL = (L ? std::min(1.0, gPlantL.lockAge / (double)kLockRampFrames) : 0.0);
                    double rampR = (R ? std::min(1.0, gPlantR.lockAge / (double)kLockRampFrames) : 0.0);
                    double ramp = 0.0; int cntGain = 0;
                    if (L) { ramp += rampL; ++cntGain; }
                    if (R) { ramp += rampR; ++cntGain; }
                    if (cntGain > 0) ramp /= cntGain;

                    // 기본 게인에 램프 반영
                    double dynGain = kLockGain * ramp;

                    // 게인 + 스텝 제한
                    double stepX = std::clamp(eX_sens * dynGain, -kMaxStepXY, kMaxStepXY); // Sensor X → VTK X
                    double stepY = std::clamp(eY_sens * dynGain, -kMaxStepXY, kMaxStepXY); // Sensor Y → VTK Z

                    constexpr double kMetersToVtk = 5.0;
                    std::lock_guard lk(gMutex);
                    gPosOffset.x += static_cast<float>(stepX / kMetersToVtk);
                    gPosOffset.y += static_cast<float>(stepY / kMetersToVtk);
                }
            }
        }


        if (mSensorIdx == 0) mRenWin->Render();

    }

private:
    // 부모 센서 인덱스 반환
    int getParentSensorIndex(int sensorIdx) {
        switch (static_cast<SensorIndex>(sensorIdx)) {
        case SensorIndex::LEFT_LEG:     return static_cast<int>(SensorIndex::PELVIS);
        case SensorIndex::LEFT_CALF:   return static_cast<int>(SensorIndex::LEFT_LEG);
        case SensorIndex::LEFT_FOOT:   return static_cast<int>(SensorIndex::LEFT_CALF);
        case SensorIndex::RIGHT_LEG:   return static_cast<int>(SensorIndex::PELVIS);
        case SensorIndex::RIGHT_CALF:  return static_cast<int>(SensorIndex::RIGHT_LEG);
        case SensorIndex::RIGHT_FOOT:  return static_cast<int>(SensorIndex::RIGHT_CALF);
        default: return -1;
        }
    }

    // ★★★ 관절 연결점의 Y 오프셋 (부모와의 연결 지점) ★★★
    double getJointOffsetY(int sensorIdx) {
        switch (static_cast<SensorIndex>(sensorIdx)) {
        case SensorIndex::LEFT_LEG:     return -5;   // 고관절에서 약간 아래
        case SensorIndex::LEFT_CALF:    return -12;  // 허벅지 하단 (무릎)
        case SensorIndex::LEFT_FOOT:    return -10;  // 종아리 하단 (발목)
        case SensorIndex::RIGHT_LEG:    return -5;   // 고관절에서 약간 아래
        case SensorIndex::RIGHT_CALF:   return -12;  // 허벅지 하단 (무릎)
        case SensorIndex::RIGHT_FOOT:   return -10;  // 종아리 하단 (발목)
        default: return 0;
        }
    }

    // ★★★ 관절 중심까지의 오프셋 (연결점 → 기하학적 중심) ★★★
    double getCenterOffsetY(int sensorIdx) {
        switch (static_cast<SensorIndex>(sensorIdx)) {
        case SensorIndex::LEFT_LEG:     return -5;   // 허벅지 중심 (길이 14의 절반)
        case SensorIndex::LEFT_CALF:    return -6; // 종아리 중심 (길이 13의 절반)
        case SensorIndex::LEFT_FOOT:    return -1.5; // 발 중심 (높이 3의 절반)
        case SensorIndex::RIGHT_LEG:    return -5;   // 허벅지 중심
        case SensorIndex::RIGHT_CALF:   return -6; // 종아리 중심  
        case SensorIndex::RIGHT_FOOT:   return -1.5; // 발 중심
        default: return 0;
        }
    }

    vtkActor* mActor{ nullptr };
    vtkTransform* mTransform{ nullptr };
    double mPosX{ 0 }, mPosY{ 0 };
    int mSensorIdx{ 0 };
    vtkRenderWindow* mRenWin{ nullptr };
    vtkTransform* mParent{ nullptr };
};

/* ======================  BASE B MATRICES  ================================ */
static void initBaseTransforms() {
    gBaseTransforms[(int)SensorIndex::PELVIS] = { 1,0,0,0, 0,0,1,0, 0,1,0,0, 0,0,0,1 };
    gBaseTransforms[(int)SensorIndex::LEFT_LEG] = { 0,1,0,0, 0,0,1,0, 1,0,0,0, 0,0,0,1 };
    gBaseTransforms[(int)SensorIndex::LEFT_CALF] = { 0,1,0,0, 0,0,1,0, 1,0,0,0, 0,0,0,1 };
    gBaseTransforms[(int)SensorIndex::LEFT_FOOT] = { 0,1,0,0, 0,0,1,0, -1,0,0,0, 0,0,0,1 };
    gBaseTransforms[(int)SensorIndex::RIGHT_LEG] = { 0,1,0,0, 0,0,1,0, -1,0,0,0, 0,0,0,1 };
    gBaseTransforms[(int)SensorIndex::RIGHT_CALF] = { 0,1,0,0, 0,0,1,0, -1,0,0,0, 0,0,0,1 };
    gBaseTransforms[(int)SensorIndex::RIGHT_FOOT] = { 0,1,0,0, 0,0,1,0, -1,0,0,0, 0,0,0,1 };
}

/* ======================  MAIN  =========================================== */
int main() {
    initBaseTransforms();
    startNetwork();

    auto colors = vtkSmartPointer<vtkNamedColors>::New();
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->SetBackground(colors->GetColor3d("White").GetData());

    auto renderWin = vtkSmartPointer<vtkRenderWindow>::New();
    renderWin->SetWindowName("VTK Motion Capture");
    renderWin->SetSize(700, 900);
    renderWin->AddRenderer(renderer);

    auto iren = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    iren->SetRenderWindow(renderWin);

    /* --- floor grid setup --- */
    auto plane = vtkSmartPointer<vtkPlaneSource>::New();
    //int floorY = -49;
    plane->SetOrigin(-1000, kFloorY, -1000);
    plane->SetPoint1(1000, kFloorY, -1000);
    plane->SetPoint2(-1000, kFloorY, 1000);
    plane->SetXResolution(10); plane->SetYResolution(10); plane->Update();

    // 1) 체커 이미지 생성 (프로시저럴)
    int texSize = 1024;      // 텍스처 해상도
    int tilePx = 512;        // 한 칸 픽셀 크기
    auto checker = vtkSmartPointer<vtkImageCanvasSource2D>::New();
    checker->SetExtent(0, texSize - 1, 0, texSize - 1, 0, 0);
    checker->SetNumberOfScalarComponents(3);
    checker->SetScalarTypeToUnsignedChar();
    for (int y = 0; y < texSize; y += tilePx) {
        for (int x = 0; x < texSize; x += tilePx) {
            bool dark = ((x / tilePx + y / tilePx) % 2) == 0;
            // 칸 색상(밝기 튜닝 가능)
            if (dark) checker->SetDrawColor(60, 60, 60);
            else      checker->SetDrawColor(200, 200, 200);
            checker->FillBox(x, x + tilePx - 1, y, y + tilePx - 1);
        }
    }

    // 2) UV 스케일(바닥 면적 대비 몇 번 반복할지)
    auto tcX = 20.0;  // X 방향 반복 횟수
    auto tcY = 20.0;  // Y 방향 반복 횟수
    auto tcoordsXform = vtkSmartPointer<vtkTransformTextureCoords>::New();
    tcoordsXform->SetInputConnection(plane->GetOutputPort()); // 기존 plane 사용
    tcoordsXform->SetScale(tcX, tcY, 1);

    // 3) 텍스처 객체 생성 및 적용
    auto tex = vtkSmartPointer<vtkTexture>::New();
    tex->SetInputConnection(checker->GetOutputPort());
    tex->InterpolateOff();  // 픽셀 또렷하게
    tex->RepeatOn();        // UV 반복 허용

    

    auto floorMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    floorMapper->SetInputConnection(plane->GetOutputPort());

    auto floorSurface = vtkSmartPointer<vtkActor>::New();
    floorSurface->SetMapper(floorMapper);
    floorSurface->GetProperty()->SetColor(0.92, 0.92, 0.92);
    floorSurface->GetProperty()->SetSpecular(0);
    floorSurface->GetProperty()->LightingOff(); floorSurface->PickableOff();

    vtkMapper::SetResolveCoincidentTopologyToPolygonOffset();

    auto floorGrid = vtkSmartPointer<vtkActor>::New();
    floorGrid->SetMapper(floorMapper);
    floorGrid->GetProperty()->SetRepresentationToWireframe();
    floorGrid->GetProperty()->SetColor(0.3, 0.3, 0.3);
    floorGrid->GetProperty()->SetLineWidth(1.0); floorGrid->GetProperty()->LightingOff(); floorGrid->PickableOff();
    renderer->AddActor(floorSurface); renderer->AddActor(floorGrid);

    floorMapper->SetInputConnection(tcoordsXform->GetOutputPort()); // ← 중요
    floorSurface->SetTexture(tex);
    floorSurface->GetProperty()->SetColor(1, 1, 1); // 텍스처 보이도록 흰색

    // camera
    auto cam = renderer->GetActiveCamera();
    cam->SetPosition(200, 200, 200); cam->SetFocalPoint(0, 0, 0); cam->SetViewUp(0, 1, 0);
    cam->Azimuth(-45); cam->Elevation(-30); cam->Zoom(1.2);
    renderer->ResetCameraClippingRange();

    /* --- ★★★ 수정된 계층구조: 단순화된 변환 ★★★ --- */
    vtkNew<vtkTransform> lowerBodyTr;     // PELVIS
    vtkNew<vtkTransform> leftLegTr;       // LEFT_LEG
    vtkNew<vtkTransform> leftCalfTr;      // LEFT_CALF  
    vtkNew<vtkTransform> leftFootTr;      // LEFT_FOOT
    vtkNew<vtkTransform> rightLegTr;      // RIGHT_LEG
    vtkNew<vtkTransform> rightCalfTr;     // RIGHT_CALF
    vtkNew<vtkTransform> rightFootTr;     // RIGHT_FOOT

    // ★★★ 중간 변환 추가: 좌우 다리 분리용 ★★★
    vtkNew<vtkTransform> leftHipTr;   // 왼쪽 고관절 위치
    vtkNew<vtkTransform> rightHipTr;  // 오른쪽 고관절 위치

    // 골반에서 좌우로 분리
    leftHipTr->SetInput(lowerBodyTr);
    leftHipTr->Translate(-5, -5, 0);  // 왼쪽으로 8, 아래로 4

    rightHipTr->SetInput(lowerBodyTr);
    rightHipTr->Translate(5, -5, 0);   // 오른쪽으로 8, 아래로 4

    // ★★★ 계층 구조 설정 (부모-자식 관계) ★★★
    leftLegTr->SetInput(leftHipTr);        // 왼쪽 허벅지 ← 왼쪽 고관절
    leftCalfTr->SetInput(leftLegTr);       // 왼쪽 종아리 ← 왼쪽 허벅지  
    leftFootTr->SetInput(leftCalfTr);      // 왼쪽 발 ← 왼쪽 종아리

    rightLegTr->SetInput(rightHipTr);      // 오른쪽 허벅지 ← 오른쪽 고관절
    rightCalfTr->SetInput(rightLegTr);     // 오른쪽 종아리 ← 오른쪽 허벅지
    rightFootTr->SetInput(rightCalfTr);    // 오른쪽 발 ← 오른쪽 종아리

    /* --- ★★★ 관절 구(sphere) 추가 ★★★ --- */
    vtkNew<vtkTransform> leftPelvisTr, rightPelvisTr;     // 골반 관절
    vtkNew<vtkTransform> leftKneeTr, rightKneeTr;         // 무릎 관절  
    vtkNew<vtkTransform> leftAnkleTr, rightAnkleTr;       // 발목 관절

    // 골반 관절 위치 (고관절)
    leftPelvisTr->SetInput(leftHipTr);
    leftPelvisTr->Translate(0, -1, 0);

    rightPelvisTr->SetInput(rightHipTr);
    rightPelvisTr->Translate(0, -1, 0);

    // 무릎 관절 위치
    leftKneeTr->SetInput(leftLegTr);
    leftKneeTr->Translate(0, -9, 0);

    rightKneeTr->SetInput(rightLegTr);
    rightKneeTr->Translate(0, -9, 0);

    // 발목 관절 위치  
    leftAnkleTr->SetInput(leftCalfTr);
    leftAnkleTr->Translate(0, -8, 0);

    rightAnkleTr->SetInput(rightCalfTr);
    rightAnkleTr->Translate(0, -8, 0);

    /* --- 기하학적 액터들 생성 (구 포함) --- */
    auto lowerBodyActor = makeActor(makeCylinder(11, 9), lowerBodyTr, colors);

    // 골반 관절 구들
    auto leftPelvisActor = makeActor(makeSphere(4.7), leftPelvisTr, colors);
    auto rightPelvisActor = makeActor(makeSphere(4.7), rightPelvisTr, colors);

    // 허벅지
    auto leftLegActor = makeActor(makeCylinder(4.3, 14), leftLegTr, colors);
    auto rightLegActor = makeActor(makeCylinder(4.3, 14), rightLegTr, colors);

    // 무릎 관절 구들
    auto leftKneeActor = makeActor(makeSphere(4), leftKneeTr, colors);
    auto rightKneeActor = makeActor(makeSphere(4), rightKneeTr, colors);

    // 종아리
    auto leftCalfActor = makeActor(makeCylinder(3, 13), leftCalfTr, colors);
    auto rightCalfActor = makeActor(makeCylinder(3, 13), rightCalfTr, colors);

    // 발목 관절 구들
    auto leftAnkleActor = makeActor(makeSphere(3), leftAnkleTr, colors);
    auto rightAnkleActor = makeActor(makeSphere(3), rightAnkleTr, colors);

    // 발
    auto leftFootActor = makeActor(makeCube(6, 3, 12), leftFootTr, colors, "SkyBlue");
    auto rightFootActor = makeActor(makeCube(6, 3, 12), rightFootTr, colors);

    for (auto act : { lowerBodyActor, leftPelvisActor, rightPelvisActor,
                     leftLegActor, rightLegActor, leftKneeActor, rightKneeActor,
                     leftCalfActor, rightCalfActor, leftAnkleActor, rightAnkleActor,
                     leftFootActor, rightFootActor })
        renderer->AddActor(act);

    /* --- ★★★ 수정된 타이머 콜백 (좌우 분리 적용) ★★★ --- */
    iren->AddObserver(vtkCommand::TimerEvent,
        SensorTimerCallback::New(lowerBodyActor, lowerBodyTr, 0, 0,
            static_cast<int>(SensorIndex::PELVIS), renderWin));

    iren->AddObserver(vtkCommand::TimerEvent,
        SensorTimerCallback::New(leftLegActor, leftLegTr, 0, 0,
            static_cast<int>(SensorIndex::LEFT_LEG), renderWin, leftHipTr));

    iren->AddObserver(vtkCommand::TimerEvent,
        SensorTimerCallback::New(leftCalfActor, leftCalfTr, 0, 0,
            static_cast<int>(SensorIndex::LEFT_CALF), renderWin, leftLegTr));

    iren->AddObserver(vtkCommand::TimerEvent,
        SensorTimerCallback::New(leftFootActor, leftFootTr, 0, 0,
            static_cast<int>(SensorIndex::LEFT_FOOT), renderWin, leftCalfTr));

    iren->AddObserver(vtkCommand::TimerEvent,
        SensorTimerCallback::New(rightLegActor, rightLegTr, 0, 0,
            static_cast<int>(SensorIndex::RIGHT_LEG), renderWin, rightHipTr));

    iren->AddObserver(vtkCommand::TimerEvent,
        SensorTimerCallback::New(rightCalfActor, rightCalfTr, 0, 0,
            static_cast<int>(SensorIndex::RIGHT_CALF), renderWin, rightLegTr));

    iren->AddObserver(vtkCommand::TimerEvent,
        SensorTimerCallback::New(rightFootActor, rightFootTr, 0, 0,
            static_cast<int>(SensorIndex::RIGHT_FOOT), renderWin, rightCalfTr));

    /* --- keypress handler --- */
    iren->AddObserver(vtkCommand::KeyPressEvent, KeyPressCallback::New());

    iren->Initialize();
    iren->CreateRepeatingTimer(kTimerIntervalMs);
    renderWin->Render();
    iren->Start();
    return 0;
}