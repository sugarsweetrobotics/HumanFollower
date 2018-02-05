#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdint.h>


#include <vector>

double rand_double() {
  return (double)rand() / (RAND_MAX);
}

double rand_normal(const double mu, const double sigma) {
  return mu + sigma * (sqrt( -2.0*log(rand_double()) ) * sin(2.0*M_PI*rand_double()));
}


class position {
public:
  position() {}

  position(double x, double y): x(x), y(y) {}


  position(const position& p): x(p.x), y(p.y) {}
  void operator=(const position& p) {
    x = p.x; y = p.y;
  }

public:
  double x, y;
};


class particle {
public:
  particle() {}

  particle(double x, double y, double weight=0): x(x), y(y), w(weight) {}

  particle(const particle& p): x(p.x), y(p.y), w(p.w) {}
  void operator=(const particle& p) {
    x = p.x; y = p.y; w = p.w;
  }

  ~particle() {}

public:
  double x;
  double y;
  double w;
};

class particle_list {
public:
  particle_list() {}

  particle_list(uint32_t num_particles, double sx, double sy) {
    for(uint32_t i = 0;i < num_particles;i++) {
      particles.push_back(particle(rand_normal(0.0, sx), rand_normal(0.0, sy)));
    }
    max_weight = -1;
    sum_weight = 0;
  }

  ~particle_list() {}

  void copyFrom(const particle_list& pl) {
    sum_weight = 0;
    particles.clear();
    for(auto p : pl.particles) {
      particles.push_back(p);
    }
    max_weight = pl.max_weight;
    sum_weight = pl.sum_weight;
  }
  
  particle_list(const particle_list& pl) {
    copyFrom(pl);
  }

  void operator=(const particle_list& pl) {
    copyFrom(pl);
  }

public:
  std::vector<particle> particles;
  double max_weight;
  double sum_weight;
};

class velocity2d {
public:
  velocity2d(double vx, double vy, double va) : vx(vx), vy(vy), va(va) {}
public:
  double vx;
  double vy;
  double va;
};

class motion {
public:
  velocity2d v;
  double dt;
  double sx;
  double sy;
public:
  motion(const velocity2d v, double dt, double sx, double sy):
    v(v), dt(dt), sx(sx), sy(sy) {}
};

class measurement {
public:
  measurement() {}
  measurement(const measurement& m): p(m.p), sx(m.sx), sy(m.sy) {}
  void operator=(const measurement& m) {
    p = m.p; sx = m.sx; sy = m.sy;
  }

  ~measurement() {}

public:
  position p;
  double sx;
  double sy;
};

const particle apply_motion(const particle& pp, const motion& m) {
  particle p;
  p.x = pp.x + m.v.vx * m.dt + rand_normal(0.0, m.sx);
  p.y = pp.y + m.v.vy * m.dt + rand_normal(0.0, m.sy);
  return p;
}


const particle weight(const particle& p, const measurement& m) {
  particle pp(p);
  double dx = (m.p.x - p.x) / m.sx;
  double dy = (m.p.y - p.y) / m.sy;

  pp.w = sqrt(dx * dx + dy * dy);
  return pp;
}

template<class T, const particle (*Func)(const particle&, const T &)>
const particle_list map_particles(const particle_list& pl, const T& arg) { //, const particle (*Func)(const particle&, const T&)) {
  particle_list plist;
  double max_weight = -1;
  plist.sum_weight = 0;
  for (auto p : pl.particles) {
    auto pp = Func(p, arg);
    max_weight = pp.w > max_weight ? pp.w : max_weight;
    plist.sum_weight += pp.w;
    plist.particles.push_back(pp);
  }
  plist.max_weight = max_weight;
  return plist;
}

template<class T, const T (*Func)(const particle_list& pl, const particle&, const T &)>
const T reduce_particles(const particle_list& pl) {
  T t;
  for(auto p : pl.particles) {
    t = Func(pl, p, t);
  }
  return t;
}

const particle_list apply_motion(const particle_list& pl, const motion& m) {
  return map_particles<motion, &apply_motion>(pl, m);
}

const particle_list resampling(const particle_list& pl, const measurement& m) {
  return map_particles<measurement, &weight>(pl, m);
}


const position sum_position(const particle_list& pl, const particle& p, const position& pos) {
  position pp;
  pp.x = pos.x + p.x / pl.sum_weight;
  pp.y = pos.y + p.y / pl.sum_weight;
  return pp;
}

const position mean_position(const particle_list& pl) {
  return reduce_particles<position, &sum_position>(pl);
}


