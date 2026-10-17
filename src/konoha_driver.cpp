// Alur program:
//   1. buildStrokes()  -> kumpulan goresan (huruf, daun, spiral) berupa daftar titik
//   2. Tiap goresan jadi 2 pekerjaan: [pen up + pindah ke awal] lalu [pen down + gambar]
//   3. Node menjalankan pekerjaan satu per satu: atur pen -> kejar titik-titik jalur -> lanjut
#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/set_pen.hpp"

using Twist  = geometry_msgs::msg::Twist;
using Pose   = turtlesim::msg::Pose;
using SetPen = turtlesim::srv::SetPen;
using namespace std::chrono_literals;

// ===== KONFIGURASI =====
constexpr double PI = 3.14159265358979;
constexpr double STEP = 0.10;           // jarak antar titik pada jalur
constexpr double LOOK = 0.20;           // jarak look-ahead (pure pursuit)
constexpr double TOL  = 0.04;           // toleransi dianggap sampai
constexpr double VMAX = 2.0, WMAX = 10.0;

constexpr double CX = 5.544;            // tengah kanvas turtlesim
constexpr double TW = 0.6, TH = 1.2;    // lebar & tinggi huruf
constexpr double TY = 7.0, TP = 0.85;   // posisi y bawah huruf & jarak antar huruf
constexpr double IMG_CY = 4.8, IMG_SCALE = 0.030;   // skala gambar daun (piksel -> koordinat)
constexpr double FIN_X = 5.544, FIN_Y = 1.5;        // posisi akhir turtle

// ===== GEOMETRI =====
struct Pt { double x, y; };
using Path = std::vector<Pt>;

void append(Path& a, const Path& b) { a.insert(a.end(), b.begin(), b.end()); }

// Busur elips dari sudut a0 ke a1
Path arc(double cx, double cy, double rx, double ry, double a0, double a1, int n = 20) {
  Path p;
  for (int k = 0; k <= n; k++) {
    double a = a0 + (a1 - a0) * k / n;
    p.push_back({cx + rx * std::cos(a), cy + ry * std::sin(a)});
  }
  return p;
}

// Rapatkan titik tiap segmen supaya jaraknya <= STEP
Path densify(const Path& in) {
  Path out{in.front()};
  for (size_t i = 1; i < in.size(); i++) {
    double dx = in[i].x - in[i-1].x, dy = in[i].y - in[i-1].y;
    int n = std::max(1, (int)std::ceil(std::hypot(dx, dy) / STEP));
    for (int k = 1; k <= n; k++) out.push_back({in[i-1].x + dx * k / n, in[i-1].y + dy * k / n});
  }
  return out;
}

// ===== GAMBAR =====
// Goresan untuk satu huruf, x = sisi kiri huruf
std::vector<Path> letter(char c, double x) {
  double l = x, r = x + TW, cx = x + TW / 2;
  double b = TY, t = TY + TH, m = TY + TH / 2;   // bawah, atas, tengah
  switch (c) {
    case 'K': return { Path{{l,t},{l,b}}, Path{{r,t},{l,m},{r,b}} };
    case 'N': return { Path{{l,b},{l,t},{r,b},{r,t}} };
    case 'H': return { Path{{l,t},{l,b}}, Path{{r,t},{r,b}}, Path{{l,m},{r,m}} };
    case 'E': return { Path{{r,t},{l,t},{l,b},{r,b}}, Path{{l,m},{r-0.1,m}} };
    case 'O': return { arc(cx, m, TW/2, TH/2, PI/2, PI/2 + 2*PI, 40) };
    case 'G': return { arc(cx, m, TW/2, TH/2, PI/4, 2*PI, 30), Path{{r,m},{cx,m}} };
    case 'A': {
      double y = b + 0.35 * TH, in = 0.35 * TW / 2;
      return { Path{{l,b},{cx,t},{r,b}}, Path{{l+in,y},{r-in,y}} };
    }
    case 'U': {
      Path u{{l,t},{l,b+TW/2}};
      append(u, arc(cx, b+TW/2, TW/2, TW/2, PI, 2*PI));
      u.push_back({r,t});
      return { u };
    }
    case 'R': {
      double rb = TW / 2, bb = t - 2*rb;
      Path s{{l,b},{l,t},{cx,t}};
      append(s, arc(cx, t-rb, rb, rb, PI/2, -PI/2));
      s.push_back({l,bb});
      return { s, Path{{cx,bb},{r,b}} };
    }
  }
  return {};
}

// Titik hasil tracing gambar (piksel) -> koordinat turtlesim
Path fromPixels(Path p) {
  for (auto& q : p) q = {CX + (q.x - 357.5) * IMG_SCALE, IMG_CY - (q.y - 154.0) * IMG_SCALE};
  return p;
}

std::vector<Path> buildStrokes() {
  std::vector<Path> s;

  // Teks, dipusatkan di tengah kanvas
  std::string text = "KONOHAGAKURE";
  double x0 = CX - ((text.size() - 1) * TP + TW) / 2;
  for (size_t i = 0; i < text.size(); i++)
    for (auto& p : letter(text[i], x0 + i * TP)) s.push_back(p);

  // Daun: ekor + busur luar
  s.push_back(fromPixels({
    {345,212},{318,215},{290,217},{262,216},{240,218},{225,220},
    {238,212},{250,205},{262,197},{275,185},{285,175},{300,162},{315,148},{330,128},
    {343,115},{360,110},{380,108},{405,111},{425,120},{440,138},{447,146},{457,141},
    {465,130},{473,120},{482,105},{490,88}}));

  // Spiral
  s.push_back(fromPixels({
    {343,115},{325,133},{315,155},{313,178},{322,200},{345,212},{370,219},{395,218},
    {415,205},{430,188},{433,168},{427,152},{412,144},{395,140},{375,138},{362,148},
    {358,165},{365,178},{378,183},{388,177},{390,170}}));
  return s;
}

// ===== KONTROL: pure pursuit =====
// Hitung kecepatan menuju target (dx, dy). slow = perlambat saat mendekati titik akhir.
Twist pursue(double dx, double dy, double th, bool slow) {
  double d = std::hypot(dx, dy);
  double a = std::remainder(std::atan2(dy, dx) - th, 2 * PI);   // selisih sudut [-pi, pi]
  Twist c;
  if (std::fabs(a) > 0.8) {                    // sudut terlalu besar: putar di tempat
    c.angular.z = std::clamp(6.0 * a, -WMAX, WMAX);
  } else {                                     // maju sambil belok
    double v = VMAX * std::pow(std::cos(a), 3);
    if (slow) v = std::min(v, 2.5 * d);
    c.linear.x = v;
    c.angular.z = std::clamp(2.0 * v * std::sin(a) / std::max(d, 0.05), -WMAX, WMAX);
  }
  return c;
}

// ===== NODE ROS 2 =====
struct Job { Path path; bool pen_down; };      // satu pekerjaan: ikuti jalur dengan pen naik/turun

class Konoha : public rclcpp::Node {
public:
  Konoha() : Node("konoha_driver") {
    pub_ = create_publisher<Twist>("/turtle1/cmd_vel", 10);
    pen_ = create_client<SetPen>("/turtle1/set_pen");
    sub_ = create_subscription<Pose>("/turtle1/pose", 10, [this](Pose::SharedPtr m) {
      x_ = m->x; y_ = m->y; th_ = m->theta; got_pose_ = true;
    });

    // Tiap goresan: pindah dulu (pen up), lalu gambar (pen down). Terakhir ke posisi akhir.
    for (auto& s : buildStrokes()) {
      Path p = densify(s);
      jobs_.push_back({Path{p.front()}, false});
      jobs_.push_back({p, true});
    }
    jobs_.push_back({Path{Pt{FIN_X, FIN_Y}}, false});

    timer_ = create_wall_timer(20ms, [this] { tick(); });
    RCLCPP_INFO(get_logger(), "=== KONOHA START (%zu goresan) ===", jobs_.size() / 2);
  }

private:
  enum State { SET_PEN, WAIT_PEN, DRIVE };

  void tick() {
    if (!got_pose_ || !pen_->service_is_ready() || i_ >= jobs_.size()) return;
    if (state_ == SET_PEN) { setPen(jobs_[i_].pen_down); state_ = WAIT_PEN; }
    else if (state_ == DRIVE) drive();
    // WAIT_PEN: menunggu respon service, callback yang pindah ke DRIVE
  }

  void setPen(bool down) {
    auto r = std::make_shared<SetPen::Request>();
    r->r = r->g = r->b = 255; r->width = 3; r->off = !down;
    pen_->async_send_request(r, [this](rclcpp::Client<SetPen>::SharedFuture) { state_ = DRIVE; });
  }

  // Kejar titik di jalur: lompati titik yang sudah dekat (look-ahead), berhenti di titik terakhir
  void drive() {
    const Path& path = jobs_[i_].path;
    size_t last = path.size() - 1;
    while (idx_ < last && std::hypot(path[idx_].x - x_, path[idx_].y - y_) < LOOK) idx_++;

    double dx = path[idx_].x - x_, dy = path[idx_].y - y_;
    bool isEnd = (idx_ == last);

    if (isEnd && std::hypot(dx, dy) < TOL) {   // pekerjaan selesai -> lanjut berikutnya
      pub_->publish(Twist());
      idx_ = 0; i_++; state_ = SET_PEN;
      if (i_ >= jobs_.size()) RCLCPP_INFO(get_logger(), "=== SELESAI ===");
      return;
    }
    pub_->publish(pursue(dx, dy, th_, isEnd));
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<Twist>::SharedPtr pub_;
  rclcpp::Subscription<Pose>::SharedPtr sub_;
  rclcpp::Client<SetPen>::SharedPtr pen_;

  std::vector<Job> jobs_;
  size_t i_ = 0, idx_ = 0;                     // pekerjaan saat ini & titik target di jalurnya
  State state_ = SET_PEN;
  bool got_pose_ = false;
  double x_ = 5.544, y_ = 5.544, th_ = 0.0;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Konoha>());
  rclcpp::shutdown();
  return 0;
}