// Sonum : prototype de route droite avec bâtiments et hérisson original en low-poly.
// Contrôles : Haut = accélérer, Bas = freiner, Gauche/Droite = se déplacer, OK = sauter.
//
// Si le build indique une erreur sur un #include, garde les #include d'origine
// du modèle NumWorks et ajoute seulement #include <stdint.h>.

#include <eadk.h>
#include <stdint.h>

using namespace EADK;

namespace {

constexpr int SCREEN_W = 320;
constexpr int SCREEN_H = 240;
constexpr int HORIZON = 120;
constexpr float FOCAL = 200.0f;
constexpr float CAM_HEIGHT = 2.6f;
constexpr float ROAD_HALF = 4.0f;
constexpr float VMAX = 3.0f;
constexpr float BUILD_SPACING = 16.0f;
constexpr float DRAW_DIST = 260.0f;

uint16_t fb[SCREEN_W * SCREEN_H];

struct Vec3 { float x, y, z; };
struct P2 { float x, y; };
struct Tri { Vec3 a, b, c; uint16_t col; };
struct Proj { P2 p[3]; float depth; uint16_t col; };

// État du jeu
float camX = 0, camZ = 0;
float sonicX = 0, sonicY = 0, vy = 0, speed = 0;
bool onGround = true;

constexpr uint16_t rgb(int r, int g, int b) {
  return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3));
}

uint16_t shade(uint16_t c, float k) {
  int r = ((c >> 11) & 31) << 3;
  int g = ((c >> 5) & 63) << 2;
  int b = (c & 31) << 3;
  r = (int)(r * k);
  g = (int)(g * k);
  b = (int)(b * k);
  if (r > 255) r = 255;
  if (g > 255) g = 255;
  if (b > 255) b = 255;
  return rgb(r, g, b);
}

float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

float minOf3(float a, float b, float c) {
  float m = a < b ? a : b;
  return m < c ? m : c;
}

float maxOf3(float a, float b, float c) {
  float m = a > b ? a : b;
  return m > c ? m : c;
}

// Projection 3D vers écran. Renvoie false si le point est derrière la caméra.
bool project(float x, float y, float z, P2& out, float& dz) {
  dz = z - camZ;
  if (dz < 0.5f) return false;
  out.x = SCREEN_W / 2 + FOCAL * (x - camX) / dz;
  out.y = HORIZON - FOCAL * (y - CAM_HEIGHT) / dz;
  return true;
}

void fillRect(int x0, int y0, int x1, int y1, uint16_t c) {
  if (x0 < 0) x0 = 0;
  if (y0 < 0) y0 = 0;
  if (x1 > SCREEN_W - 1) x1 = SCREEN_W - 1;
  if (y1 > SCREEN_H - 1) y1 = SCREEN_H - 1;
  for (int y = y0; y <= y1; ++y)
    for (int x = x0; x <= x1; ++x)
      fb[y * SCREEN_W + x] = c;
}

float edge(P2 a, P2 b, float px, float py) {
  return (b.x - a.x) * (py - a.y) - (b.y - a.y) * (px - a.x);
}

void fillTri(P2 a, P2 b, P2 c, uint16_t col) {
  int x0 = (int)clampf(minOf3(a.x, b.x, c.x), 0.0f, SCREEN_W - 1.0f);
  int x1 = (int)clampf(maxOf3(a.x, b.x, c.x), 0.0f, SCREEN_W - 1.0f);
  int y0 = (int)clampf(minOf3(a.y, b.y, c.y), 0.0f, SCREEN_H - 1.0f);
  int y1 = (int)clampf(maxOf3(a.y, b.y, c.y), 0.0f, SCREEN_H - 1.0f);
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      float px = x + 0.5f, py = y + 0.5f;
      float w0 = edge(b, c, px, py);
      float w1 = edge(c, a, px, py);
      float w2 = edge(a, b, px, py);
      bool inside = (w0 >= 0 && w1 >= 0 && w2 >= 0) || (w0 <= 0 && w1 <= 0 && w2 <= 0);
      if (inside) fb[y * SCREEN_W + x] = col;
    }
  }
}

// Ciel et route : chaque ligne de l'écran est calculée à une profondeur donnée.
void drawSkyAndGround() {
  for (int y = 0; y < SCREEN_H; ++y) {
    if (y < HORIZON) {
      int t = y * 255 / HORIZON;
      uint16_t c = rgb(110 + t / 4, 170 + t / 6, 235 + t / 20);
      for (int x = 0; x < SCREEN_W; ++x) fb[y * SCREEN_W + x] = c;
    } else {
      float dz = FOCAL * CAM_HEIGHT / (y - HORIZON + 0.5f);
      float worldZ = camZ + dz;
      int stripe = ((int)(worldZ / 4.0f)) & 1;
      float lx = SCREEN_W / 2 + FOCAL * (-ROAD_HALF - camX) / dz;
      float rx = SCREEN_W / 2 + FOCAL * (ROAD_HALF - camX) / dz;
      uint16_t road = stripe ? rgb(96, 96, 104) : rgb(84, 84, 92);
      uint16_t grass = stripe ? rgb(60, 150, 70) : rgb(50, 135, 60);
      for (int x = 0; x < SCREEN_W; ++x)
        fb[y * SCREEN_W + x] = (x >= lx && x <= rx) ? road : grass;
    }
  }
}

uint32_t hashBuilding(int n, int side) {
  uint32_t h = (uint32_t)n * 2654435761u ^ (uint32_t)(side + 2) * 40503u;
  h ^= h >> 15;
  h *= 0x2C1B3C6Du;
  h ^= h >> 12;
  return h;
}

// Bâtiments des deux côtés, du plus loin au plus proche.
void drawBuildings() {
  int nFar = (int)((camZ + DRAW_DIST) / BUILD_SPACING);
  int nNear = (int)(camZ / BUILD_SPACING);
  for (int n = nFar; n >= nNear; --n) {
    float z = n * BUILD_SPACING;
    for (int side = -1; side <= 1; side += 2) {
      uint32_t h = hashBuilding(n, side);
      float hgt = 7.0f + (float)(h % 18);
      uint16_t wall = rgb(90 + (h % 60), 90 + ((h >> 6) % 50), 110 + ((h >> 12) % 60));

      P2 bottomIn, topIn, bottomOut;
      float dz;
      if (!project(side * 6.0f, 0, z, bottomIn, dz)) continue;
      if (dz > DRAW_DIST) continue;
      project(side * 6.0f, hgt, z, topIn, dz);
      project(side * 10.0f, 0, z, bottomOut, dz);

      int x0 = (int)minOf3(bottomIn.x, bottomOut.x, bottomIn.x);
      int x1 = (int)maxOf3(bottomIn.x, bottomOut.x, bottomIn.x);
      int y0 = (int)topIn.y;
      int y1 = (int)bottomIn.y;
      fillRect(x0, y0, x1, y1, wall);

      // Fenêtres simples
      if (x1 - x0 > 8) {
        uint16_t win = shade(wall, 1.35f);
        for (int x = x0 + 2; x <= x1 - 2; x += 4)
          fillRect(x, y0 + (y1 - y0) / 4, x + 1, y1 - (y1 - y0) / 4, win);
      }
    }
  }
}

// Hérisson low-poly original : une liste de triangles en coordonnées du monde.
Tri tris[64];
int triCount = 0;

void addTri(Vec3 a, Vec3 b, Vec3 c, uint16_t col) {
  if (triCount < 64) tris[triCount++] = {a, b, c, col};
}

// Octaèdre : 8 triangles, ombrage plat selon l'orientation de chaque face.
void addOcta(Vec3 c, float hx, float hy, float hz, uint16_t col) {
  for (int i = -1; i <= 1; i += 2)
    for (int j = -1; j <= 1; j += 2)
      for (int k = -1; k <= 1; k += 2) {
        float light = 0.62f + (i > 0 ? 0.18f : 0) + (j > 0 ? 0.2f : 0) + (k > 0 ? 0.06f : 0);
        addTri({c.x + i * hx, c.y, c.z},
               {c.x, c.y + j * hy, c.z},
               {c.x, c.y, c.z + k * hz},
               shade(col, light));
      }
}

void buildSonic() {
  triCount = 0;
  float sx = sonicX;
  float sz = camZ + 6.0f;
  float by = sonicY;
  const uint16_t BLUE = rgb(30, 112, 224);
  const uint16_t DBLUE = rgb(20, 80, 170);
  const uint16_t RED = rgb(220, 40, 40);

  addOcta({sx, by + 1.0f, sz}, 0.7f, 0.8f, 0.7f, BLUE);           // corps
  addOcta({sx, by + 1.85f, sz + 0.05f}, 0.5f, 0.45f, 0.5f, BLUE); // tête

  // Piquants à l'arrière de la tête
  float dxs[3] = {-0.45f, 0.0f, 0.45f};
  for (int s = 0; s < 3; ++s) {
    float tipY = by + 2.7f + (s == 1 ? 0.2f : 0.0f);
    addTri({sx - 0.4f, by + 2.0f, sz - 0.3f},
           {sx + 0.4f, by + 2.0f, sz - 0.3f},
           {sx + dxs[s], tipY, sz - 0.9f},
           DBLUE);
  }

  // Chaussures
  addOcta({sx - 0.35f, by + 0.18f, sz + 0.1f}, 0.3f, 0.18f, 0.35f, RED);
  addOcta({sx + 0.35f, by + 0.18f, sz + 0.1f}, 0.3f, 0.18f, 0.35f, RED);
}

void drawSonic() {
  buildSonic();
  Proj pr[64];
  int n = 0;
  for (int t = 0; t < triCount; ++t) {
    Vec3 v[3] = {tris[t].a, tris[t].b, tris[t].c};
    Proj item;
    bool ok = true;
    float dsum = 0;
    for (int i = 0; i < 3; ++i) {
      float dz;
      if (!project(v[i].x, v[i].y, v[i].z, item.p[i], dz)) { ok = false; break; }
      dsum += dz;
    }
    if (!ok) continue;
    item.depth = dsum / 3.0f;
    item.col = tris[t].col;
    pr[n++] = item;
  }
  // Tri par profondeur décroissante (les plus loin en premier)
  for (int i = 1; i < n; ++i) {
    Proj key = pr[i];
    int j = i - 1;
    while (j >= 0 && pr[j].depth < key.depth) {
      pr[j + 1] = pr[j];
      --j;
    }
    pr[j + 1] = key;
  }
  for (int i = 0; i < n; ++i)
    fillTri(pr[i].p[0], pr[i].p[1], pr[i].p[2], pr[i].col);
}

void drawHud() {
  int len = (int)(speed / VMAX * 150.0f);
  fillRect(4, 4, 4 + len, 10, rgb(255, 210, 0));
}

void update(const Keyboard::State& kbd) {
  if (kbd.keyDown(Keyboard::Key::Up)) speed += 0.03f;
  else speed *= 0.993f;
  if (kbd.keyDown(Keyboard::Key::Down)) speed *= 0.93f;
  speed = clampf(speed, 0.0f, VMAX);

  if (kbd.keyDown(Keyboard::Key::Left)) sonicX -= 0.12f;
  if (kbd.keyDown(Keyboard::Key::Right)) sonicX += 0.12f;
  sonicX = clampf(sonicX, -3.4f, 3.4f);

  if (kbd.keyDown(Keyboard::Key::OK) && onGround) {
    vy = 0.42f;
    onGround = false;
  }
  if (!onGround) {
    vy -= 0.03f;
    sonicY += vy;
    if (sonicY <= 0) {
      sonicY = 0;
      vy = 0;
      onGround = true;
    }
  }

  camZ += speed;
  camX += (sonicX * 0.6f - camX) * 0.1f;
}

}  // namespace

void eadk_main() {
  while (true) {
    Keyboard::State kbd = Keyboard::scan();
    update(kbd);
    drawSkyAndGround();
    drawBuildings();
    drawSonic();
    drawHud();
    Display::pushRect(Display::Rect(0, 0, SCREEN_W, SCREEN_H),
                      reinterpret_cast<const Display::Color*>(fb));
  }
}
