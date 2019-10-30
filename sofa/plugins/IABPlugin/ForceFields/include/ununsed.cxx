
    // const VecCoord& X = x.getValue();
    // VecDeriv& force = *f.beginEdit();
    // size_t nFrames = X.size();
    //
    // std::copy(X.begin(), X.end(), m_pos.begin());
    //
    // const Real& coef = d_coeff.getValue();
    // const unsigned int& start = d_startIndex.getValue();
    //
    // for(size_t n = 0 ; n < nFrames - (start + 1) ; ++n) {
    //     const Pos& x0Current = X[n + 0 + start].getCenter();
    //     const Pos& x1Current = X[n + 1 + start].getCenter();
    //
    //     const Pos& x0Rest = m_restPos[n + 0 + start].getCenter();
    //     const Pos& x1Rest = m_restPos[n + 1 + start].getCenter();
    //
    //     const Quaternion& q0Current = X[n + 0 + start].getOrientation();
    //     const Quaternion& q1Current = X[n + 1 + start].getOrientation();
    //     const Quaternion& q0Rest = m_restPos[n + 0 + start].getOrientation();
    //     const Quaternion& q1Rest = m_restPos[n + 1 + start].getOrientation();
    //
    //     // compute x1 local rigid position and rotation (can be precomputed)
    //     const Pos& x1l0Rest = q0Rest.inverseRotate(x1Rest - x0Rest);
    //     const Quaternion& q1l0Rest = q0Rest.inverse() * q1Rest;
    //
    //     // compute x1 position w.r.t. x0 frame
    //     const Pos& x1l0Current = q0Current.inverseRotate(x1Current - x0Current);
    //
    //     // compute the difference between rigid and real positions and orientations
    //     const Pos& dx = x1l0Rest - x1l0Current;
    //     //    Quaternion qDiff = q0Current.inverse() * q1th.inverse() * q1Current;
    //     Quaternion dummy;
    //     Quaternion q1l0  = q0Current.inverse() * q1Current;
    //     Quaternion qdiff = q1l0 * q1l0Rest.inverse();
    //     qdiff.normalize();
    //
    //     Vec3 dq = dummy.angularDisplacement(q1l0Rest, q1l0);
    //     Vec6 dX1(dx, dq);
    //
    //     // test: verification that we obtain the good values for linear and angular displacements in local frames
    //     Transform World_H_X0_rest(x0Rest,q0Rest);
    //     Transform World_H_X1_rest(x1Rest,q1Rest);
    //     Transform X0_H_X1_Rest = World_H_X0_rest.inversed() * World_H_X1_rest;
    //
    //     Transform World_H_X0_current(x0Current,q0Current);
    //     Transform World_H_X1_current(x1Current,q1Current);
    //     Transform X0_H_X1_Current = World_H_X0_current.inversed() * World_H_X1_current;
    //
    //     /// compute the angular displacement:
    //     SpatialVector UinX0= X0_H_X1_Current.CreateSpatialVector() - X0_H_X1_Rest.CreateSpatialVector();
    //     Vec6 Uloc;
    //     for (unsigned int i=0; i<3; i++)
    //     {
    //         Uloc[i] = UinX0.getLinearVelocity()[i];
    //         Uloc[i+3] = UinX0.getAngularVelocity()[i];
    //     }
    //     Vec6 Floc = m_CInv[n] * Uloc;
    //     Vec3 f_loc(Floc.ptr());
    //     Vec3 tau_loc(Floc.ptr() + 3);
    //
    //     SpatialVector F1atX1inX0(f_loc, tau_loc);
    //
    //     bool rest=true;
    //     SpatialVector F1atX1inX1,F0atX0inX0;
    //     if(rest){
    //         F1atX1inX1.setForce( X0_H_X1_Rest.backProjectVector( F1atX1inX0.getForce()) );
    //         F1atX1inX1.setTorque( X0_H_X1_Rest.backProjectVector( F1atX1inX0.getTorque()) )  ;
    //         F0atX0inX0 = X0_H_X1_Rest*F1atX1inX1;
    //     }
    //     else
    //     {
    //         F1atX1inX1.setForce( X0_H_X1_Current.backProjectVector( F1atX1inX0.getForce()) );
    //         F1atX1inX1.setTorque( X0_H_X1_Current.backProjectVector( F1atX1inX0.getTorque()) )  ;
    //         F0atX0inX0 = X0_H_X1_Current*F1atX1inX1;
    //     }
    //
    //     SpatialVector F0atX0inWorld(World_H_X0_current.projectVector(F0atX0inX0.getForce()), World_H_X0_current.projectVector(F0atX0inX0.getTorque()))  ;
    //     SpatialVector F1atX1inWorld(World_H_X0_current.projectVector(F1atX1inX0.getForce()),  World_H_X0_current.projectVector(F1atX1inX0.getTorque()));
    //
    //
    //     // compute x1 forces in x0's frame and rotate them back to global coordinates
    //     Vec6 f1l0 = m_CInv[n] * dX1;
    //
    //     Vec3 F1(f1l0.ptr());
    //     Vec3 tau1(f1l0.ptr() + 3);
    //     F1 = q0Current.rotate(F1);
    //     tau1 = q0Current.rotate(tau1);
    //
    //
    //     // compute transport matrix
    //     Vec3 p0p1;
    //     if(rest)
    //         p0p1= q0Rest.inverseRotate(x1Rest - x0Rest); // p0^p1 in local frame
    //     else
    //         p0p1= q0Current.inverseRotate(x1Current - x0Current); // p0^p1 in local frame
    //
    //     Mat66 H = Mat66::Identity();
    //     H(3, 1) = -p0p1.z();
    //     H(3, 2) = p0p1.y();
    //     H(4, 0) = p0p1.z();
    //     H(4, 2) = -p0p1.x();
    //     H(5, 0) = -p0p1.y();
    //     H(5, 1) = p0p1.x();
    //
    //
    //     H = -H;
    //
    //     // compute f0
    //     Vec6 f0l0 = H * f1l0;
    //     Vec3 F0(f0l0.ptr());
    //     Vec3 tau0(f0l0.ptr() + 3);
    //
    //     F0 = q0Current.rotate(F0);
    //     tau0 = q0Current.rotate(tau0);
    //
    //
    //     Vec6 f0( F0atX0inWorld.getForce(),  F0atX0inWorld.getTorque());
    //     Vec6 f1(-F1atX1inWorld.getForce(), -F1atX1inWorld.getTorque());
    //
    //     force[n + 0 + start] += f0 * coef;
    //     force[n + 1 + start] += f1 * coef;
    //
    //
    //     Mat66 block = H * m_CInv[n];
    //     m_K[n + start].setsub(0, 6, block);
    //     m_K[n + start].setsub(6, 0, block.transposed());
    //
    //     block =  H * m_CInv[n] * H.transposed();
    //     m_K[n + start].setsub(0, 0, block);
    //     m_K[n + start].setsub(6, 6, m_CInv[n]);
    //
    //
    //     // build rotation matrix 4 3x3 blocks on diagonal
    //     Mat33 Rn;
    //     q0Current.toMatrix(Rn);
    //     Mat12x12 R(.0);
    //     R.setsub(0, 0, Rn);
    //     R.setsub(3, 3, Rn);
    //     R.setsub(6, 6, Rn);
    //     R.setsub(9, 9, Rn);
    //
    //     m_K[n + start] = -R * m_K[n + start] * R.transposed() * coef; // modified : wasn't negated
    // }
    // f.endEdit();

        /*
        // see https://www.boost.org/doc/libs/1_67_0/libs/numeric/odeint/doc/html/boost_numeric_odeint/odeint_in_detail/integrate_functions.html
        From https://github.com/headmyshoulder/odeint-v2/blob/master/examples/harmonic_oscillator.cpp#L165-#L171
        */
            /*[integrate_adapt_full
            state_type stress_rr(1);
            stress_rr[0] = m_Ro; // start at Ro
        double abs_err = 1.0e-10 , rel_err = 1.0e-6 , a_x = 1.0 , a_dxdt = 1.0;
        // controlled_stepper_type controlled_stepper(
        //     default_error_checker< double , range_algebra , default_operations >( abs_err , rel_err , a_x , a_dxdt ) );
            // integrate_adaptive( controlled_stepper , ref_2_cur , stress_rr , m_ri , m_ro , 0.01 );
        controlled_stepper_type controlled_stepper;
        integrate_adaptive( controlled_stepper , ref_2_cur , stress_rr , 0.0 , 10.0 , 0.01 );
        */
           /* output */
// from integrand.inl

  //I have used 24|b R is the unknown variable being integrated from ri to ro
// template<typename value_type>
// void radial_stress_r2c<value_type>::operator() (const state_type &x , state_type &dxdt , const double /* t */ ) // see equation 25 in ContinuumI paper
// {
//   dxdt[0] = -1*(2*C1_*(x[0]/std::pow(Ro_, 2) \
//                 -std::pow(Ro_, 4)/std::pow(x[0], 5)) \
//                 +2*C2_*(std::pow(x[0], 3)/std::pow(Ro_, 4) \
//                 - std::pow(Ro_, 2)/std::pow(x[0], 3)));
// }

// from integrand.inl
template<typename value_type>
pressure_r2c<value_type>::pressure_r2c(const value_type& Ri, const value_type& Ro,
                          const value_type& ri,
                          const value_type& C1, const value_type& C2)
                          : Ri_(Ri), Ro_(Ro), ri_(ri), C1_(C1), C2_(C2)
                          { }

template<typename value_type>
pressure_r2c<value_type>::~pressure_r2c()
{
  // destructor implementation
};

//I have used 26|a R is the unknown variable being integrated from ri to ro
template<typename value_type>
value_type pressure_r2c<value_type>::operator() (const value_type& r) const // see equation 25 in ContinuumI paper
{
  return 2*C1_*(r/std::pow(Ro_, 2) \
                -std::pow(Ro_, 4)/std::pow(r, 5)) \
                +2*C2_*(std::pow(r, 3)/std::pow(Ro_, 4) \
                - std::pow(Ro_, 2)/std::pow(r, 3));
}

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
        std::cout << x[0]  << "\n";
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


using state = radial_stress_r2c<double>;
using error_stepper_type = runge_kutta_cash_karp54< state_type >;
using controlled_stepper_type = controlled_runge_kutta< error_stepper_type >;
