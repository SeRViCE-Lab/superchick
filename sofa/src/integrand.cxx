#include "integration.h"
#include <cmath>

// see equation 25 in ContinuumI paper
template<typename value_type>
class radial_stress_contact_free{
  public:
    radial_stress_contact_free(const value_type& Ri,
                               const value_type& Ro, , const value_type& ri,
                               const value_type& C1, const value_type& C2,
                               const value_type& p)
                                : Ri_(Ri), Ro_(Ro), ri_(ri), C1_(C1), C2_(C2),
                                p_(p) { }
    value_type operator() (const value_type& R) const // see equation 25 in ContinuumI paper
    {
      //return the integrand expression
      const value_type& ro_ = std::cbrt(std::pow(Ro_, 3) + std::pow(ri_, 3) - std::pow(Ri_, 3));
      return 2*C1_*(1/ro_ - std::pow(R, 6)/std::pow(ro, 7))- \
                  2*C2_*(std::pow(R, 6)/std::pow(r^7)-ro_/std::pow(R, 2));
    }

  private:
    const value_type Ri_; // internal radius in current configuration
    const value_type Ro_; // internal radius in reference configuration
    const value_type ri_; // internal radius in reference configuration
    const value_type C1_; // young's modulus of internal IAB wall
    const value_type C1_; // young's modulus of internal IAB wall
    const value_type p_; // hydrostatic pressure, if not zero

};

// see equation 26 in ContinuumI paper
template<typename value_type>
class internal_pressure_contact_free{
  public:
    internal_pressure_contact_free(const value_type& Ri,
                                   const value_type& Ro, , const value_type& ri,
                                   const value_type& C1, const value_type& C2,
                                   const value_type& p)
                                  : Ri_(Ri), Ro_(Ro), ri_(ri), C1_(C1), C2_(C2),
                                  p_(p) { }
    value_type operator() (const value_type& R) const // see equation 25 in ContinuumI paper
    {
      //return the integrand expression
      const value_type& ro_ = std::cbrt(std::pow(Ro_, 3) + std::pow(ri_, 3) - std::pow(Ri_, 3));
      return 2*C1_*(1/ro_ - std::pow(R, 6)/std::pow(ro, 7))- \
                  2*C2_*(std::pow(R, 6)/std::pow(r^7)-ro_/std::pow(R, 2));
    }

  private:Thanks
    const value_type Ri_; // internal radius in current configuration
    const value_type Ro_; // internal radius in reference configuration
    const value_type ri_; // internal radius in reference configuration
    const value_type C1_; // young's modulus of internal IAB wall
    const value_type C1_; // young's modulus of internal IAB wall
    const value_type p_; // hydrostatic pressure, if not zero

};
