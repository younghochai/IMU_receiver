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


// GLM (header‑only) for vec3 ----------------------------------------------
#include <glm/glm.hpp>

// === floor clamp config ===
constexpr double kFloorY = -49.0;  // 바닥 높이(기존 floorY와 통일)
constexpr double kClampTol = 1e-3;   // 수치 떨림 방지용 아주 작은 여유

std::atomic<double> gLeftFootBottomY{ kFloorY };
std::atomic<double> gRightFootBottomY{ kFloorY };


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
constexpr int          kTimerIntervalMs = 10; // VTK timer period (10 ms)

/* ===============================  TYPES  ================================= */
struct Quaternion { double w{ 1 }, x{ 0 }, y{ 0 }, z{ 0 }; };

enum class SensorIndex : int {
    PELVIS = 0, LEFT_LEG, LEFT_CALF, LEFT_FOOT,
    RIGHT_LEG, RIGHT_CALF, RIGHT_FOOT,
    COUNT
};

/* =====================  GLOBAL STATE & SYNCHRO  =========================== */
std::array<Quaternion, kSensorCount> gLatestQuat{};         // live quats
std::array<glm::vec3, kSensorCount> gLatestPos{};          // live positions
std::array<Quaternion, kSensorCount> gCalibrationOffset{};  // per‑sensor offset

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
        if ((frameCount % 1000) == 0) printf("[NET] frames: %zu\n", frameCount);

        const float* f = reinterpret_cast<const float*>(buf.data());
        {
            std::lock_guard lk(gMutex);
            for (int i = 0; i < kSensorCount; ++i, f += 7) {
                gLatestQuat[i] = { f[0], f[1], f[2], f[3] };
                gLatestPos[i] = { f[4], f[6], -f[5] }; ///★★★ YZ 수정
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
                csv << ", sensor" << i << "_px, sensor" << i << "_py, sensor" << i << "_pz";
            }
            csv << "\n";
            // rows
            for (auto& row : gSensorLog) {
                for (size_t i = 0; i < row.size(); ++i) csv << (i ? "," : "") << row[i];
                csv << "\n";
            }
            printf("[LOG] CSV saved (%zu rows)\n", gSensorLog.size());
        }
        else if (key == "c") {
            std::lock_guard lk(gMutex); gCalibrationOffset = gLatestQuat; printf("[LOG] calibrated\n");
        }
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

        // Pelvis 위치 계산 수정
        if (mSensorIdx == static_cast<int>(SensorIndex::PELVIS)) {
            constexpr double kMetersToVtk = 5.0;

            glm::vec3 offset;
            { std::lock_guard lk(gMutex); offset = gPosOffset; }

            mTransform->Identity();
            mTransform->Translate(
                (pos.x + offset.x) * kMetersToVtk,  // X축
                (pos.y + offset.y)* 0.00001,  // Y축 (상하) - offset.z 추가!
                (pos.z + offset.z) * kMetersToVtk   // Z축 (전후)
            );
            mTransform->Concatenate(Q);

            // 디버그 출력 (필요시)
            static int frameCount = 0;
            if (++frameCount % 1000 == 0) {
                printf("[PELVIS] Raw pos=(%.3f, %.3f, %.3f), Offset=(%.3f, %.3f, %.3f), Final=(%.3f, %.3f, %.3f)\n",
                    pos.x, pos.y, pos.z,
                    offset.x, offset.y, offset.z,
                    (pos.x + offset.x) * kMetersToVtk, // 좌우
                    (pos.y + offset.y) * 0.00001,                // 위아래
                    (pos.z + offset.z) * kMetersToVtk);// 앞뒤
            }
        }
        else {
            // ★★★ 관절들: 연결점에서 회전하도록 수정

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

        //// --- (A) 이 콜백 액터가 발이면, 월드 최저점(ymin)을 기록 ---
        //if (mSensorIdx == static_cast<int>(SensorIndex::LEFT_FOOT) ||
        //    mSensorIdx == static_cast<int>(SensorIndex::RIGHT_FOOT)) {
        //    double b[6];
        //    mActor->GetBounds(b);         // [xmin, xmax, ymin, ymax, zmin, zmax] in world
        //    double bottomY = b[2];        // ymin = 발바닥 최저점
        //    if (mSensorIdx == static_cast<int>(SensorIndex::LEFT_FOOT))
        //        gLeftFootBottomY.store(bottomY, std::memory_order_relaxed);
        //    else
        //        gRightFootBottomY.store(bottomY, std::memory_order_relaxed);
        //}

        // --- (B) 프레임당 1번: 오른발 콜백에서 전신 오프셋 보정 수행 ---
        // 등록 순서가 PELVIS → ... → LEFT_FOOT → ... → RIGHT_FOOT 이므로
        // RIGHT_FOOT가 마지막이라고 가정(현재 main 등록 순서 그대로라면 OK)
        //if (mSensorIdx == static_cast<int>(SensorIndex::RIGHT_FOOT)) {
        //    const double leftB = gLeftFootBottomY.load(std::memory_order_relaxed);
        //    const double rightB = gRightFootBottomY.load(std::memory_order_relaxed);
        //    const double minBottom = std::min(leftB, rightB);

        //    // 목표: minBottom == kFloorY (한 발은 반드시 바닥에 닿게)
        //    double delta = 0.0;
        //    if (minBottom < kFloorY - kClampTol) {
        //        // 뚫림: 위로 올림
        //        delta = kFloorY - minBottom;  // +값
        //    }
        //    else if (minBottom > kFloorY + kClampTol) {
        //        // 공중부양: 아래로 내림
        //        delta = kFloorY - minBottom;  // -값
        //    }

        //    if (std::abs(delta) > 0.0) {
        //        std::lock_guard lk(gMutex);
        //        // 전신(골반 상위) 오프셋에 반영 → 다음 프레임부터 즉시 적용
        //        gPosOffset.z += static_cast<float>(delta);

        //        // (선택) 프레임당 이동량 제한으로 급격한 튐 방지
        //        // const double kMaxStep = 5.0; // 필요시 사용
        //        // gPosOffset.y += static_cast<float>(std::clamp(delta, -kMaxStep, kMaxStep));
        //    }
        //}

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

    // ★★★ 관절 연결점의 Y 오프셋 (부모와의 연결 지점)
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

    // ★★★ 관절 중심까지의 오프셋 (연결점 → 기하학적 중심)
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


/* ======================  MAIN  =========================================== */
int main() {
    startNetwork();

    auto colors = vtkSmartPointer<vtkNamedColors>::New();
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->SetBackground(colors->GetColor3d("White").GetData());

    auto renderWin = vtkSmartPointer<vtkRenderWindow>::New();
    renderWin->SetWindowName("ROBOT 7ea Sensors (C++) - Fixed Rotation Axis");
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

    /* --- ★★★ 수정된 계층구조: 단순화된 변환 --- */
    vtkNew<vtkTransform> lowerBodyTr;     // PELVIS
    vtkNew<vtkTransform> leftLegTr;       // LEFT_LEG
    vtkNew<vtkTransform> leftCalfTr;      // LEFT_CALF  
    vtkNew<vtkTransform> leftFootTr;      // LEFT_FOOT
    vtkNew<vtkTransform> rightLegTr;      // RIGHT_LEG
    vtkNew<vtkTransform> rightCalfTr;     // RIGHT_CALF
    vtkNew<vtkTransform> rightFootTr;     // RIGHT_FOOT

    // ★★★ 중간 변환 추가: 좌우 다리 분리용 
    vtkNew<vtkTransform> leftHipTr;   // 왼쪽 고관절 위치
    vtkNew<vtkTransform> rightHipTr;  // 오른쪽 고관절 위치

    // 골반에서 좌우로 분리
    leftHipTr->SetInput(lowerBodyTr);
    leftHipTr->Translate(-5, -5, 0);  // 왼쪽으로 8, 아래로 4

    rightHipTr->SetInput(lowerBodyTr);
    rightHipTr->Translate(5, -5, 0);   // 오른쪽으로 8, 아래로 4

    // ★★★ 계층 구조 설정 (부모-자식 관계) 
    leftLegTr->SetInput(leftHipTr);        // 왼쪽 허벅지 ← 왼쪽 고관절
    leftCalfTr->SetInput(leftLegTr);       // 왼쪽 종아리 ← 왼쪽 허벅지  
    leftFootTr->SetInput(leftCalfTr);      // 왼쪽 발 ← 왼쪽 종아리

    rightLegTr->SetInput(rightHipTr);      // 오른쪽 허벅지 ← 오른쪽 고관절
    rightCalfTr->SetInput(rightLegTr);     // 오른쪽 종아리 ← 오른쪽 허벅지
    rightFootTr->SetInput(rightCalfTr);    // 오른쪽 발 ← 오른쪽 종아리

    /* --- ★★★ 관절 구(sphere) 추가  --- */
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

    /* --- ★★★ 수정된 타이머 콜백 (좌우 분리 적용) --- */
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