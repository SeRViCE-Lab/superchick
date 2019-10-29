#include <cmath>

// https://www.boost.org/doc/libs/1_67_0/libs/numeric/odeint/doc/html/boost_numeric_odeint/getting_started/usage__compilation__headers.html
#include <boost/numeric/odeint.hpp> // all steppers
// #include <boost/numeric/odeint/algebra/XYZ.hpp> //all algebras
// #include <boost/numeric/odeint/integrate/XYZ.hpp> // all integrate routines

using namespace boost::numeric::odeint;
// //use boost ode solver to P and Stress:: lot more stable than trapezoidal rule
// runge_kutta_dopri5<state,double,state,double,vector_space_algebra> stepper;
// runge_kutta_dopri5() stepper;

// https://github.com/headmyshoulder/odeint-v2/blob/master/examples/harmonic_oscillator.cpp
/* The type of container used to hold the state vector */
typedef std::vector< double > state_type;

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

    // value_type operator() (const value_type& r) const; // see equation 25 in ContinuumI paper
    void operator() (const state_type &x , state_type &dxdt , const double /* t */ ) // see equation 25 in ContinuumI paper
    {
      dxdt[0] = -1*(2*C1_*(x[0]/std::pow(Ro_, 2) - std::pow(Ro_, 4)/std::pow(x[0], 5)) \
          +2*C2_*(std::pow(x[0], 3)/std::pow(Ro_, 4)-std::pow(Ro_, 2)/std::pow(x[0], 3)));
    }


private:
  const value_type Ro_, Ri_, ri_, C1_, C2_;
};

//[ integrate_observer
struct push_back_state_and_time
{
    std::vector< state_type >& m_states;
    std::vector< double >& m_times;

    push_back_state_and_time( std::vector< state_type > &states , std::vector< double > &times )
    : m_states( states ) , m_times( times ) { }

    void operator()( const state_type &x , double t )
    {
        m_states.push_back( x );
        m_times.push_back( t );
    }
};

struct write_state
{
    void operator()( const state_type &x ) const
    {
        std::cout << x[0] << "\t" << x[1] << "\n";
    }
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

using state = radial_stress_r2c<double>;
using error_stepper_type = runge_kutta_cash_karp54< state_type >;
using controlled_stepper_type = controlled_runge_kutta< error_stepper_type >;
