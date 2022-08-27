#include "graphics.hpp"

void draw_state(sf::RenderWindow& window, std::vector<Boid> const& state)
{
  // creates an array of vertices defining a Triangles primitive (i.e. a set of
  // unconnected triangles)
  sf::VertexArray triangles(sf::Triangles);

  for (Boid const& boid : state) {
    // predators are represented as bigger triangles
    double const scale_fac{(boid.is_pred()) ? 1.5 : 1.};
    int constexpr half_base{2};   // half of the base of the triangle
    int constexpr half_height{3}; // half of the height of the triangle
    sf::Color grey{105, 105, 105, 255};
    sf::Color orange{235, 149, 50, 255};

    // each triangle is defined by three vertices
    sf::Vertex top;
    top.position = sf::Vector2f(boid.position().x(),
                                boid.position().y() + scale_fac * half_height);
    top.color    = (boid.is_pred()) ? sf::Color::Red : sf::Color::Black;

    sf::Vertex lr;
    lr.position = sf::Vector2f(boid.position().x() - scale_fac * half_base,
                               boid.position().y() - scale_fac * half_height);
    sf::Vertex ll;
    ll.position = sf::Vector2f(boid.position().x() + scale_fac * half_base,
                               boid.position().y() - scale_fac * half_height);
    lr.color = ll.color = (boid.is_pred()) ? orange : grey;

    // rotating triangle to match boid's velocity's direction:
    double angle{((boid.velocity().x() < 0) ? (1.) : (-1.)) * (180. / pi)
                 * std::acos(boid.velocity().y() / norm(boid.velocity()))};
    sf::Transform rotation;
    rotation.rotate(angle, boid.position().x(), boid.position().y());
    // rotates vertex of [angle] degrees (clockwise order)
    top.position = rotation.transformPoint(top.position);
    lr.position  = rotation.transformPoint(lr.position);
    ll.position  = rotation.transformPoint(ll.position);

    // inserting vertices in the array
    triangles.append(top);
    triangles.append(lr);
    triangles.append(ll);
  }

  window.draw(triangles);
}

auto evolve(Flock& flock, Parameters const& pars)
{
  for (int i{0}; i != pars.get_steps(); ++i) {
    flock.evolve(pars);
  }
  return flock.state();
}

void game_loop(sf::RenderWindow& window, Flock& flock, Parameters const& pars,
               unsigned int seed){
window.setFramerateLimit (pars.get_fps());
               }
