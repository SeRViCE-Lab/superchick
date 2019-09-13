#include <cmath>

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
//Destructor
virtual ~radial_stress_c2r();

inline value_type get_r();

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
    radial_stress_r2c(const value_type& ri,
                                             const value_type& ro,
                                             const value_type& Ri,
                                             const value_type& C1,
                                             const value_type& C2);
// Destructor
virtual ~radial_stress_r2c();
};

template<typename value_type>
class pressure_r2c
{
  public:
    pressure_r2c(const value_type& ri, const value_type& ro,
                                 const value_type& Ri, const value_type& C1,
                                 const value_type& C2);
//Destructor
virtual ~pressure_r2c();

  private:
    const value_type ri_; // internal radius in current configuration
    const value_type ro_; // external radius in current configuration
    const value_type Ri_; // internal radius in reference configuration
    const value_type C1_; // young's modulus of internal IAB wall
    const value_type C2_; // young's modulus of internal IAB wall
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

  private:
    const value_type Ri_; // internal radius in current configuration
    const value_type Ro_; // internal radius in reference configuration
    const value_type ri_; // internal radius in reference configuration
    const value_type C1_; // young's modulus of internal IAB wall
    const value_type C2_; // young's modulus of internal IAB wall
};
