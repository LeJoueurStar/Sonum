#include "eadkpp.h"
#include <stdint.h>

extern const char eadk_app_name[] __attribute__((section(".rodata.eadk_app_name"))) = "Sonum";
extern const uint32_t eadk_api_level __attribute__((section(".rodata.eadk_api_level"))) = 0;

using namespace EADK;

namespace {

constexpr int SCREEN_W = 320;
constexpr int SCREEN_H = 240;
constexpr int BAND_H = 20;
constexpr int HORIZON = 90;
constexpr float FOCAL = 200.0f;
constexpr float CAM_HEIGHT = 5.5f;
constexpr float ROAD_HALF = 4.0f;
constexpr float VMAX = 3.0f;
constexpr float BUILD_SPACING = 16.0f;
constexpr float DRAW_DIST = 260.0f;
constexpr float RING_SPACING = 12.0f;
constexpr float SEG_LEN = 60.0f;
constexpr float LEVEL_LENGTH = 2400.0f;
constexpr float CURVE_AMP = 7.0f;
constexpr float SONIC_AHEAD = 14.0f;
constexpr float SC = 0.7f;

constexpr int Z_FLAT = 0;
constexpr int Z_RAMP = 1;
constexpr int Z_TRAMP = 2;
constexpr int Z_BOOST = 3;
constexpr int Z_ENEMY = 4;

enum class GameScreen { Title, Play, Goal };
GameScreen screen = GameScreen::Title;

struct Vec3 { float x, y, z; };
struct P2 { float x, y; };
struct Tri { Vec3 a, b, c; uint16_t col; };
struct Proj { P2 p[3]; float depth; uint16_t col; };

// État du jeu
float camX = 0, camZ = 0;
float sonicX = 0, sonicY = 0, vy = 0, speed = 0, phase = 0, sonicDZ = 0;
bool onGround = true;
int rings = 0;
int attackE = -1;
int attackT = 0;
int hitCool = 0;
int collected[64];   // collected[n & 63] == n : anneau n ramassé
int killedId[64];    // killedId[e & 63] == e : ennemi e détruit

// Tampon d'une seule bande horizontale (320 x 20 pixels, 12,8 Ko)
uint16_t fb[SCREEN_W * BAND_H];
int bandY0 = 0;

// Liste de triangles pour Sonic et les ennemis
Tri tris[96];
int triCount = 0;

constexpr uint16_t rgb(int r, int g, int b) {
  return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3));
}

const uint16_t BLUE = rgb(30, 112, 224);
const uint16_t DBLUE = rgb(20, 80, 170);
const uint16_t RED = rgb(220, 40, 40);
const uint16_t DRED = rgb(120, 20, 20);
const uint16_t WHITE = rgb(255, 255, 255);
const uint16_t YELLOW = rgb(255, 210, 0);
const uint16_t GREEN = rgb(80, 220, 80);
const uint16_t DARK = rgb(20, 20, 40);
const uint16_t MAG = rgb(200, 60, 170);
const uint16_t CYAN = rgb(60, 210, 230);
const uint16_t RAMP = rgb(170, 140, 95);

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

float absf(float v) {
  return v < 0 ? -v : v;
}

float minOf3(float a, float b, float c) {
  float m = a < b ? a : b;
  return m < c ? m : c;
}

float maxOf3(float a, float b, float c) {
  float m = a > b ? a : b;
  return m > c ? m : c;
}

// Onde triangulaire de période 1, valeurs dans [-1, 1]. Pour x >= 0. Pas de libm.
float tri(float x) {
  float f = x - (float)(int)x;
  return f < 0.5f ? 4.0f * f - 1.0f : 3.0f - 4.0f * f;
}

// Décalage latéral de la route : virages doux
float roadCenter(float z) {
  return CURVE_AMP * tri(z * 0.004f);
}

int zoneIdx(float z) {
  return (int)(z / SEG_LEN);
}

// Type de zone : déterministe, les deux premières zones sont plates
int zoneType(int i) {
  if (i < 2) return Z_FLAT;
  uint32_t h = (uint32_t)i * 2654435761u;
  h ^= h >> 13;
  h *= 0x5BD1E995u;
  h ^= h >> 15;
  static const int table[6] = {Z_FLAT, Z_FLAT, Z_RAMP, Z_TRAMP, Z_BOOST, Z_ENEMY};
  return table[h % 6];
}

// Hauteur d'une rampe en forme de bosse, u = position dans la zone
float rampH(float u) {
  if (u < 10.0f || u > 40.0f) return 0.0f;
  float d = u - 25.0f;
  if (d < 0) d = -d;
  float h = 3.0f * (1.0f - d / 15.0f);
  return h > 0 ? h : 0.0f;
}

float roadH(float z) {
  int i = zoneIdx(z);
  if (zoneType(i) != Z_RAMP) return 0.0f;
  return rampH(z - i * SEG_LEN);
}

// Vrai si le point est sur le tremplin (dernière partie de la zone)
bool inPad(float z) {
  int i = zoneIdx(z);
  if (zoneType(i) != Z_TRAMP) return false;
  float u = z - i * SEG_LEN;
  return u >= 20.0f && u <= 40.0f;
}

// Écrit un pixel seulement s'il est dans la bande courante
inline void plot(int x, int y, uint16_t c) {
  if (x < 0 || x >= SCREEN_W) return;
  if (y < bandY0 || y >= bandY0 + BAND_H) return;
  fb[(y - bandY0) * SCREEN_W + x] = c;
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
  if (x1 > SCREEN_W - 1) x1 = SCREEN_W - 1;
  if (y0 < bandY0) y0 = bandY0;
  if (y1 > bandY0 + BAND_H - 1) y1 = bandY0 + BAND_H - 1;
  for (int y = y0; y <= y1; ++y)
    for (int x = x0; x <= x1; ++x)
      fb[(y - bandY0) * SCREEN_W + x] = c;
}

// Triangle plein, limité à la bande courante
void fillTri(P2 a, P2 b, P2 c, uint16_t col) {
  float lo = (float)bandY0;
  float hi = (float)(bandY0 + BAND_H - 1);
  int x0 = (int)clampf(minOf3(a.x, b.x, c.x), 0.0f, SCREEN_W - 1.0f);
  int x1 = (int)clampf(maxOf3(a.x, b.x, c.x), 0.0f, SCREEN_W - 1.0f);
  int y0 = (int)clampf(minOf3(a.y, b.y, c.y), lo, hi);
  int y1 = (int)clampf(maxOf3(a.y, b.y, c.y), lo, hi);
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      float px = x + 0.5f, py = y + 0.5f;
      float w0 = (c.x - b.x) * (py - b.y) - (c.y - b.y) * (px - b.x);
      float w1 = (a.x - c.x) * (py - c.y) - (a.y - c.y) * (px - c.x);
      float w2 = (b.x - a.x) * (py - a.y) - (b.y - a.y) * (px - a.x);
      bool inside = (w0 >= 0 && w1 >= 0 && w2 >= 0) || (w0 <= 0 && w1 <= 0 && w2 <= 0);
      if (inside) plot(x, y, col);
    }
  }
}

// Police 3x5 : chaque ligne est un motif de 3 bits (bit de gauche = colonne 0)
const char FONT_CHARS[] = "ABDEGHILMNOPRSTUVXY0123456789-: ";
const uint8_t FONT[][5] = {
  {0b010,0b101,0b111,0b101,0b101}, // A
  {0b110,0b101,0b110,0b101,0b110}, // B
  {0b110,0b101,0b101,0b101,0b110}, // D
  {0b111,0b100,0b110,0b100,0b111}, // E
  {0b111,0b100,0b101,0b101,0b111}, // G
  {0b101,0b101,0b111,0b101,0b101}, // H
  {0b111,0b010,0b010,0b010,0b111}, // I
  {0b100,0b100,0b100,0b100,0b111}, // L
  {0b101,0b111,0b111,0b101,0b101}, // M
  {0b101,0b111,0b111,0b111,0b101}, // N
  {0b111,0b101,0b101,0b101,0b111}, // O
  {0b110,0b101,0b110,0b100,0b100}, // P
  {0b110,0b101,0b110,0b101,0b101}, // R
  {0b111,0b100,0b111,0b001,0b111}, // S
  {0b111,0b010,0b010,0b010,0b010}, // T
  {0b101,0b101,0b101,0b101,0b111}, // U
  {0b101,0b101,0b101,0b101,0b010}, // V
  {0b101,0b101,0b010,0b101,0b101}, // X
  {0b101,0b101,0b010,0b010,0b010}, // Y
  {0b111,0b101,0b101,0b101,0b111}, // 0
  {0b010,0b110,0b010,0b010,0b111}, // 1
  {0b111,0b001,0b111,0b100,0b111}, // 2
  {0b111,0b001,0b111,0b001,0b111}, // 3
  {0b101,0b101,0b111,0b001,0b001}, // 4
  {0b111,0b100,0b111,0b001,0b111}, // 5
  {0b111,0b100,0b111,0b101,0b111}, // 6
  {0b111,0b001,0b010,0b010,0b010}, // 7
  {0b111,0b101,0b111,0b101,0b111}, // 8
  {0b111,0b101,0b111,0b001,0b111}, // 9
  {0b000,0b000,0b111,0b000,0b000}, // -
  {0b000,0b010,0b000,0b010,0b000}, // :
  {0b000,0b000,0b000,0b000,0b000}, // espace
};

const uint8_t* glyph(char c) {
  for (int i = 0; FONT_CHARS[i]; ++i)
    if (FONT_CHARS[i] == c) return FONT[i];
  return FONT[31];
}

void drawText(int x, int y, const char* s, int scale, uint16_t col) {
  for (; *s; ++s) {
    const uint8_t* g = glyph(*s);
    for (int row = 0; row < 5; ++row)
      for (int cl = 0; cl < 3; ++cl)
        if (g[row] & (4 >> cl)) {
          int px = x + cl * scale;
          int py = y + row * scale;
          fillRect(px, py, px + scale - 1, py + scale - 1, col);
        }
    x += 4 * scale;
  }
}

// Texte centré avec une ombre pour rester lisible
void drawTextCentered(int y, const char* s, int scale, uint16_t col) {
  int len = 0;
  while (s[len]) ++len;
  int w = len * 4 * scale - scale;
  int x = (SCREEN_W - w) / 2;
  drawText(x + 2, y + 2, s, scale, DARK);
  drawText(x, y, s, scale, col);
}

void intToStr(int v, char* out) {
  char t[12];
  int n = 0;
  if (v == 0) t[n++] = '0';
  while (v > 0 && n < 11) {
    t[n++] = (char)('0' + v % 10);
    v /= 10;
  }
  int i = 0;
  while (n > 0) out[i++] = t[--n];
  out[i] = 0;
}

// Construit "ANNEAUX 12"
void ringText(char* out) {
  const char* label = "ANNEAUX ";
  int i = 0;
  while (label[i]) {
    out[i] = label[i];
    ++i;
  }
  char num[12];
  intToStr(rings, num);
  int j = 0;
  while (num[j]) out[i++] = num[j++];
  out[i] = 0;
}

bool isKilled(int e) { return killedId[e & 63] == e; }
void markKilled(int e) { killedId[e & 63] = e; }
float enemyZ(int e) { return e * SEG_LEN + 30.0f; }
float enemyX(int e) { return roadCenter(enemyZ(e)) + ((e & 1) ? 2.2f : -2.2f); }
float enemyY(int e) { return roadH(enemyZ(e)) + 0.8f; }

// Ciel et route pour les lignes de la bande courante
void drawSkyAndGround() {
  for (int y = bandY0; y < bandY0 + BAND_H; ++y) {
    if (y < HORIZON) {
      int t = y * 255 / HORIZON;
      uint16_t c = rgb(110 + t / 4, 170 + t / 6, 235 + t / 20);
      for (int x = 0; x < SCREEN_W; ++x) plot(x, y, c);
    } else {
      float dz = FOCAL * CAM_HEIGHT / (y - HORIZON + 0.5f);
      float worldZ = camZ + dz;
      int stripe = ((int)(worldZ / 4.0f)) & 1;
      int zt = zoneType(zoneIdx(worldZ));
      uint16_t road, grass;
      if (zt == Z_TRAMP) {
        road = stripe ? MAG : rgb(140, 40, 120);
        grass = stripe ? rgb(60, 150, 70) : rgb(50, 135, 60);
      } else if (zt == Z_BOOST) {
        road = stripe ? CYAN : rgb(40, 120, 140);
        grass = stripe ? rgb(60, 150, 70) : rgb(50, 135, 60);
      } else {
        road = stripe ? rgb(96, 96, 104) : rgb(84, 84, 92);
        grass = stripe ? rgb(60, 150, 70) : rgb(50, 135, 60);
      }
      float cxw = roadCenter(worldZ);
      float lx = SCREEN_W / 2 + FOCAL * (cxw - ROAD_HALF - camX) / dz;
      float rx = SCREEN_W / 2 + FOCAL * (cxw + ROAD_HALF - camX) / dz;
      for (int x = 0; x < SCREEN_W; ++x)
        plot(x, y, (x >= lx && x <= rx) ? road : grass);
    }
  }
}

// Rampes dessinées en polygones, avec leur hauteur
void drawRamps() {
  int i0 = zoneIdx(camZ);
  int i1 = zoneIdx(camZ + DRAW_DIST);
  for (int i = i1; i >= i0; --i) {
    if (zoneType(i) != Z_RAMP) continue;
    for (int s = 0; s < 8; ++s) {
      float u0 = 10.0f + s * 3.75f;
      float u1 = u0 + 3.75f;
      float z0 = i * SEG_LEN + u0;
      float z1 = i * SEG_LEN + u1;
      float h0 = rampH(u0);
      float h1 = rampH(u1);
      float c0 = roadCenter(z0);
      float c1 = roadCenter(z1);
      P2 pl0, pr0, pl1, pr1;
      float dz;
      if (!project(c0 - ROAD_HALF, h0, z0, pl0, dz)) continue;
      if (!project(c0 + ROAD_HALF, h0, z0, pr0, dz)) continue;
      if (!project(c1 - ROAD_HALF, h1, z1, pl1, dz)) continue;
      if (!project(c1 + ROAD_HALF, h1, z1, pr1, dz)) continue;
      fillTri(pl0, pr0, pl1, RAMP);
      fillTri(pr0, pr1, pl1, RAMP);
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

// Bâtiments des deux côtés, suivant la courbe de la route
void drawBuildings() {
  int nFar = (int)((camZ + DRAW_DIST) / BUILD_SPACING);
  int nNear = (int)(camZ / BUILD_SPACING);
  for (int n = nFar; n >= nNear; --n) {
    float z = n * BUILD_SPACING;
    float cx = roadCenter(z);
    for (int side = -1; side <= 1; side += 2) {
      uint32_t h = hashBuilding(n, side);
      float hgt = 7.0f + (float)(h % 18);
      uint16_t wall = rgb(90 + (h % 60), 90 + ((h >> 6) % 50), 110 + ((h >> 12) % 60));

      P2 bottomIn, topIn, bottomOut;
      float dz;
      if (!project(cx + side * 6.0f, 0, z, bottomIn, dz)) continue;
      if (dz > DRAW_DIST) continue;
      project(cx + side * 6.0f, hgt, z, topIn, dz);
      project(cx + side * 10.0f, 0, z, bottomOut, dz);

      int x0 = (int)minOf3(bottomIn.x, bottomOut.x, bottomIn.x);
      int x1 = (int)maxOf3(bottomIn.x, bottomOut.x, bottomIn.x);
      int y0 = (int)topIn.y;
      int y1 = (int)bottomIn.y;
      fillRect(x0, y0, x1, y1, wall);

      if (x1 - x0 > 8) {
        uint16_t win = shade(wall, 1.35f);
        for (int x = x0 + 2; x <= x1 - 2; x += 4)
          fillRect(x, y0 + (y1 - y0) / 4, x + 1, y1 - (y1 - y0) / 4, win);
      }
    }
  }
}

// Anneau : cercle jaune creux, dessiné seulement dans la bande courante
void drawRing(float wx, float wy, float wz) {
  P2 c;
  float dz;
  if (!project(wx, wy, wz, c, dz)) return;
  if (dz > DRAW_DIST) return;
  int ri = (int)(FOCAL * 0.9f / dz);
  if (ri < 2) ri = 2;
  int thick = ri / 3 + 1;
  int cx = (int)c.x;
  int cy = (int)c.y;
  int y0 = cy - ri;
  if (y0 < bandY0) y0 = bandY0;
  int y1 = cy + ri;
  if (y1 > bandY0 + BAND_H - 1) y1 = bandY0 + BAND_H - 1;
  int outer = ri * ri;
  int inner = (ri - thick) * (ri - thick);
  for (int y = y0; y <= y1; ++y) {
    int dy = y - cy;
    for (int x = cx - ri; x <= cx + ri; ++x) {
      int dx = x - cx;
      int d2 = dx * dx + dy * dy;
      if (d2 <= outer && d2 >= inner) plot(x, y, YELLOW);
    }
  }
}

void drawRings() {
  int n0 = (int)(camZ / RING_SPACING);
  int n1 = (int)((camZ + DRAW_DIST) / RING_SPACING);
  for (int n = n1; n >= n0; --n) {
    if (collected[n & 63] == n) continue;
    float zr = n * RING_SPACING;
    if (zr < camZ + 1.0f) continue;
    drawRing(roadCenter(zr), roadH(zr) + 1.3f, zr);
  }
}

void addTri(Vec3 a, Vec3 b, Vec3 c, uint16_t col) {
  if (triCount < 96) tris[triCount++] = {a, b, c, col};
}

// Octaèdre : 8 triangles, ombrage plat selon l'orientation de chaque face
void addOcta(Vec3 c, float hx, float hy, float hz, uint16_t col) {
  for (int i = -1; i <= 1; i += 2) {
    for (int j = -1; j <= 1; j += 2) {
      for (int k = -1; k <= 1; k += 2) {
        float light = 0.62f + (i > 0 ? 0.18f : 0) + (j > 0 ? 0.2f : 0) + (k > 0 ? 0.06f : 0);
        addTri({c.x + i * hx, c.y, c.z},
               {c.x, c.y + j * hy, c.z},
               {c.x, c.y, c.z + k * hz},
               shade(col, light));
      }
    }
  }
}

// Projette, trie et dessine les triangles de tris[]
void drawTris() {
  Proj pr[96];
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

// Ennemi : orbe rouge à pointe
void addEnemy(float x, float y, float z) {
  addOcta({x, y, z}, 0.5f, 0.5f, 0.5f, RED);
  addOcta({x, y + 0.9f, z}, 0.2f, 0.9f, 0.2f, DRED);
}

void drawEnemies() {
  int i0 = zoneIdx(camZ);
  int i1 = zoneIdx(camZ + DRAW_DIST);
  for (int e = i1; e >= i0; --e) {
    if (zoneType(e) != Z_ENEMY || isKilled(e)) continue;
    triCount = 0;
    addEnemy(enemyX(e), enemyY(e), enemyZ(e));
    drawTris();
  }
}

// Sonic : animation procédurale (course, rebond, boule en l'air ou en attaque)
void drawSonic() {
  const float sz = camZ + SONIC_AHEAD + sonicDZ;
  const float sx = roadCenter(sz) + sonicX;
  const float sy = sonicY;
  const float k = speed / VMAX;
  triCount = 0;

  if (!onGround || attackE >= 0) {
    float w = tri(phase * 0.8f + 0.5f);
    if (w < 0) w = -w;
    addOcta({sx, sy + 0.9f * SC, sz}, (0.2f + 0.6f * w) * SC, 0.8f * SC, (0.6f + 0.1f * w) * SC, BLUE);
    drawTris();
    return;
  }

  float bob = 0.03f * k * tri(phase * 2.0f);
  float body = sy + 1.0f * SC + bob;
  float head = sy + 1.85f * SC + bob;

  addOcta({sx, body, sz}, 0.7f * SC, 0.8f * SC, 0.7f * SC, BLUE);
  addOcta({sx, head, sz + 0.05f * SC}, 0.5f * SC, 0.45f * SC, 0.5f * SC, BLUE);

  float dxs[3] = {-0.45f, 0.0f, 0.45f};
  for (int s = 0; s < 3; ++s) {
    float tipY = head + (0.85f + (s == 1 ? 0.2f : 0.0f)) * SC;
    addTri({sx - 0.4f * SC, head, sz + 0.1f * SC},
           {sx + 0.4f * SC, head, sz + 0.1f * SC},
           {sx + dxs[s] * SC, tipY, sz + 0.7f * SC},
           DBLUE);
  }

  addOcta({sx - 0.22f * SC, head + 0.08f * SC, sz - 0.42f * SC}, 0.12f * SC, 0.14f * SC, 0.06f * SC, WHITE);
  addOcta({sx + 0.22f * SC, head + 0.08f * SC, sz - 0.42f * SC}, 0.12f * SC, 0.14f * SC, 0.06f * SC, WHITE);

  float liftL = 0.14f * k * (tri(phase) + 1.0f) * 0.5f;
  float liftR = 0.14f * k * (tri(phase + 0.5f) + 1.0f) * 0.5f;
  addOcta({sx - 0.35f * SC, sy + 0.18f * SC + liftL, sz + 0.1f * SC}, 0.3f * SC, 0.18f * SC, 0.35f * SC, RED);
  addOcta({sx + 0.35f * SC, sy + 0.18f * SC + liftR, sz + 0.1f * SC}, 0.3f * SC, 0.18f * SC, 0.35f * SC, RED);

  drawTris();
}

// Ramassage des anneaux
void collectRings(float sz, float sx) {
  int n0 = (int)(camZ / RING_SPACING);
  for (int n = n0; n <= n0 + 6; ++n) {
    float zr = n * RING_SPACING;
    if (absf(zr - sz) > 1.5f) continue;
    if (absf(roadCenter(zr) - sx) > 1.5f) continue;
    if (absf(sonicY - (roadH(zr) + 1.3f)) > 2.0f) continue;
    if (collected[n & 63] == n) continue;
    collected[n & 63] = n;
    ++rings;
  }
}

// Contact avec les ennemis : en l'air = destruction, au sol = blessure
void checkEnemies(float sz, float sx) {
  int i0 = zoneIdx(camZ);
  for (int e = i0; e <= i0 + 2; ++e) {
    if (zoneType(e) != Z_ENEMY || isKilled(e)) continue;
    if (absf(sz - enemyZ(e)) > 1.5f) continue;
    if (absf(sx - enemyX(e)) > 1.5f) continue;
    if (absf(sonicY - enemyY(e)) > 1.5f) continue;
    if (!onGround) {
      markKilled(e);
      rings += 1;
      vy = 0.3f;
      onGround = false;
    } else if (hitCool == 0 && attackE != e) {
      rings = rings > 10 ? rings - 10 : 0;
      speed = 0.4f;
      hitCool = 60;
    }
  }
}

void drawHud() {
  char buf[24];
  ringText(buf);
  drawText(7, 7, buf, 2, DARK);
  drawText(5, 5, buf, 2, YELLOW);

  fillRect(5, 30, 105, 36, DARK);
  int len = (int)(speed / VMAX * 98.0f);
  if (len > 98) len = 98;
  fillRect(6, 31, 6 + len, 35, WHITE);

  float pf = camZ / LEVEL_LENGTH;
  if (pf > 1.0f) pf = 1.0f;
  int pl = (int)(pf * 308.0f);
  fillRect(5, 226, 315, 232, DARK);
  fillRect(7, 228, 7 + pl, 230, GREEN);
}

void resetLevel() {
  camZ = 0;
  camX = roadCenter(SONIC_AHEAD);
  sonicX = 0;
  sonicY = 0;
  sonicDZ = 0;
  vy = 0;
  speed = 0;
  phase = 0;
  onGround = true;
  rings = 0;
  attackE = -1;
  attackT = 0;
  hitCool = 0;
  for (int i = 0; i < 64; ++i) {
    collected[i] = -1;
    killedId[i] = -1;
  }
}

void updatePlay(const Keyboard::State& kbd, bool okPressed) {
  float sz0 = camZ + SONIC_AHEAD + sonicDZ;
  bool boost = zoneType(zoneIdx(sz0)) == Z_BOOST;
  float vmaxNow = boost ? VMAX * 1.6f : VMAX;

  if (kbd.keyDown(Keyboard::Key::Up)) speed += 0.03f;
  else speed *= 0.993f;
  if (boost) speed += 0.02f;
  if (kbd.keyDown(Keyboard::Key::Down)) speed *= 0.93f;
  speed = clampf(speed, 0.0f, vmaxNow);

  if (kbd.keyDown(Keyboard::Key::Left)) sonicX -= 0.12f;
  if (kbd.keyDown(Keyboard::Key::Right)) sonicX += 0.12f;
  sonicX = clampf(sonicX, -ROAD_HALF * 0.8f, ROAD_HALF * 0.8f);

  // Attaque aérienne : OK en l'air vise l'ennemi le plus proche devant
  if (okPressed && !onGround && attackE < 0) {
    int best = -1;
    float bestD = 1e9f;
    int i0 = zoneIdx(camZ);
    for (int e = i0; e <= i0 + 2; ++e) {
      if (zoneType(e) != Z_ENEMY || isKilled(e)) continue;
      float d = enemyZ(e) - sz0;
      if (d > 0.0f && d < 45.0f && d < bestD) {
        bestD = d;
        best = e;
      }
    }
    if (best >= 0) {
      attackE = best;
      attackT = 60;
    }
  }

  if (attackE >= 0) {
    // Boule qui fonce vers l'ennemi visé
    attackT--;
    float ez = enemyZ(attackE);
    float ex = enemyX(attackE);
    float ey = enemyY(attackE) + 0.2f;
    float tx = ex - roadCenter(sz0);
    sonicX += clampf(tx - sonicX, -0.35f, 0.35f);
    sonicY += clampf(ey - sonicY, -0.3f, 0.3f);
    sonicDZ += clampf(ez - sz0, -0.9f, 0.9f);
    float sz = camZ + SONIC_AHEAD + sonicDZ;
    float sx = roadCenter(sz) + sonicX;
    if (absf(ez - sz) < 1.8f && absf(ex - sx) < 1.8f && absf(ey - sonicY) < 1.8f) {
      markKilled(attackE);
      rings += 1;
      attackE = -1;
      vy = 0.3f;
      onGround = false;
    } else if (attackT <= 0) {
      attackE = -1;
    }
  } else {
    float sz = camZ + SONIC_AHEAD + sonicDZ;
    float g = roadH(sz);
    if (onGround) {
      if (sonicY - g > 0.3f) {
        onGround = false;
        vy = 0.0f;
      } else {
        sonicY = g;
      }
    }
    if (okPressed && onGround) {
      vy = 0.42f;
      onGround = false;
    }
    if (!onGround) {
      vy -= 0.03f;
      sonicY += vy;
      if (sonicY <= g) {
        sonicY = g;
        vy = 0.0f;
        onGround = true;
        if (inPad(sz)) {
          vy = 0.6f;
          onGround = false;
        }
      }
    }
    sonicDZ *= 0.92f;
  }

  float sz = camZ + SONIC_AHEAD + sonicDZ;
  float sx = roadCenter(sz) + sonicX;
  if (hitCool > 0) --hitCool;
  checkEnemies(sz, sx);
  collectRings(sz, sx);

  phase += speed * 0.06f;
  camZ += speed;
  float target = roadCenter(camZ + SONIC_AHEAD) + sonicX * 0.6f;
  camX += (target - camX) * 0.1f;

  if (camZ >= LEVEL_LENGTH) screen = GameScreen::Goal;
}

void drawTitle() {
  drawTextCentered(40, "SONUM", 6, YELLOW);
  drawTextCentered(150, "APPUYEZ SUR OK", 2, WHITE);
}

void drawGoal() {
  drawTextCentered(50, "NIVEAU TERMINE", 3, WHITE);
  char buf[24];
  ringText(buf);
  drawTextCentered(110, buf, 3, YELLOW);
  drawTextCentered(190, "APPUYEZ SUR OK", 2, WHITE);
}

}  // namespace

int main(int argc, char* argv[]) {
  bool okPrev = false;
  for (int i = 0; i < 64; ++i) {
    collected[i] = -1;
    killedId[i] = -1;
  }
  while (true) {
    Keyboard::State kbd = Keyboard::scan();
    bool okDown = kbd.keyDown(Keyboard::Key::OK);
    bool okPressed = okDown && !okPrev;
    okPrev = okDown;

    if (screen == GameScreen::Title) {
      if (okPressed) {
        resetLevel();
        screen = GameScreen::Play;
      }
    } else if (screen == GameScreen::Play) {
      updatePlay(kbd, okPressed);
    } else {
      if (okPressed) screen = GameScreen::Title;
    }

    for (int b = 0; b < SCREEN_H / BAND_H; ++b) {
      bandY0 = b * BAND_H;
      drawSkyAndGround();
      if (screen == GameScreen::Title) {
        drawTitle();
      } else if (screen == GameScreen::Play) {
        drawRamps();
        drawBuildings();
        drawRings();
        drawEnemies();
        drawSonic();
        drawHud();
      } else {
        drawGoal();
      }
      Display::pushRect(Rect(0, bandY0, SCREEN_W, BAND_H),
                        reinterpret_cast<const Color*>(fb));
    }
  }
}
