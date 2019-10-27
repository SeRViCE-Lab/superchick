#include <cmath>

#include <boost/numeric/odeint/stepper/runge_kutta4.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta_dopri5.hpp>
#include <boost/numeric/odeint/algebra/vector_space_algebra.hpp>

template<typename value_type, typename function_type>
inline value_type integrator(const value_type a,
                             const value_type b,
                             const value_type atol,
                             const value_type reltol,
                             function_type func);

 template<typename value_type>
 class radial_stress_c2r
 {
 public:
       radial_stress_c2r(const value_type& Ri,
                          const value_type& Ro,
                          const value_type& ri,
                          const value_type& C1,
                          const value_type& C2);
      radial_stress_c2r(); // Copy constructor
 //Destructor
 virtual ~radial_stress_c2r();

 inline value_type get_r() const;
 value_type operator() (const value_type& R) const; // see equation 25 in ContinuumI paper


 private:
   const value_type Ri_; // internal radius in current configuration
   const value_type Ro_; // internal radius in reference configuration
   const value_type ri_; // internal radius in reference configuration
   const value_type C1_; // young's modulus of internal IAB wall
   const value_type C2_; // young's modulus of internal IAB wall
 };

template<typename value_type>
class radial_stress_r2c
{
  public:
    radial_stress_r2c(const value_type& Ri,
                       const value_type& Ro,
                       const value_type& ri,
                       const value_type& C1,
                       const value_type& C2);
    // Destructor
    virtual ~radial_stress_r2c();

    // inline value_type get_r() const;

    value_type operator() (const value_type& r) const; // see equation 25 in ContinuumI paper

private:
  const value_type Ro_, Ri_, ri_, C1_, C2_;
};

template<typename value_type>
class pressure_r2c
{
  public:
    pressure_r2c(const value_type& Ri, const value_type& Ro,
                const value_type& ri, const value_type& C1,
                const value_type& C2);
//Destructor
virtual ~pressure_r2c();
value_type operator() (const value_type& r) const; // see equation 25 in ContinuumI paper
// inline value_type get_R() const;

  private:
    const value_type Ro_, Ri_, ri_, C1_, C2_;
};

template<typename value_type>
class pressure_c2r
{
  public:
    pressure_c2r(const value_type& Ri, const value_type& Ro,
                                   const value_type& ri, const value_type& C1,
                                   const value_type& C2);
  // Destructor
  virtual ~pressure_c2r();
  value_type operator() (const value_type& R) const; // see equation 25 in ContinuumI paper
  inline value_type get_r() const;

  private:
    const value_type Ri_; // internal radius in current configuration
    const value_type Ro_; // internal radius in reference configuration
    const value_type ri_; // internal radius in reference configuration
    const value_type C1_; // young's modulus of internal IAB wall
    const value_type C2_; // young's modulus of internal IAB wall
};


using namespace boost::numeric::odeint;
using state = radial_stress_r2c<double>;
// //use boost ode solver to P and Stress:: lot more stable than trapezoidal rule
// runge_kutta_dopri5<state,double,state,double,vector_space_algebra> stepper;
runge_kutta_dopri5() stepper;
