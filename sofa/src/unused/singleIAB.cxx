/*
// // get degrees of freedom
auto states = groot->getState();
// // get mechsnical degrees of freedom
auto mechStates = groot->getMechanicalState();
auto shader = groot->getShader();
groot->printComponents();
// msg_info("SphereDeform") << "states : " << states;
// msg_info("SphereDeform") << "mechStates : " << mechStates;
msg_info("SphereDeform") << "states (DoFs): " << states; // get degrees of freedom
// msg_info("SphereDeform") << "mechStates values: " << mechStates->x.getValue(); // specified by VecXd in scene file
msg_info("SphereDeform") << "mechStates Dim: " << mechStates->getCoordDimension(); // specified by VecXd in scene file
msg_info("SphereDeform") << "mechStates Dim Derivs: " << mechStates->getDerivDimension();  // specified by VecXd in scene file
msg_info("SphereDeform") << "mechStates Template Name: " << mechStates->getTemplateName(); // returns sofa::core::behavior::BaseMechanicalState has no member named ‘readPositions’
*/
// test random rotation and translation of scene:
  // v19.06/modules/SofaGeneralDeformable/SofaGeneralDeformable_test/StiffSpringForceField_test.cpp#95

  // head_ctx->get(nearPoint->mstate1);
  // Vec3Types::Frame def_grad = pos_vecs[0].getF();
  Vec3Types::Coord zeroPos;
  // pos_vecs[0].getCPos(zeroPos)

  // we can get the indices of particles in the given bounding box:
  // see /Users/olalekanogunmolu/sofa/v19.06/SofaKernel/framework/sofa/core/behavior/MechanicalState.h#L86
  //=======================================

// accessors for positions in
➜  sofa grep -rnw . -e sofa::core::ConstVecCoordId::position
./applications/plugins/SofaDistanceGrid/components/collision/DistanceGridCollisionModel.cpp:290:        // (*rigid->read(sofa::core::ConstVecCoordId::position())->getValue())[index].writeOpenGlMatrix( m );
./applications/plugins/SofaMiscCollision/RayTriangleVisitor.cpp:75:    const DataTypes::VecCoord& x = tm->getMechanicalState()->read(sofa::core::ConstVecCoordId::position())->getValue();
./applications/plugins/PersistentContact/PersistentContactBarycentricMapping.inl:528:    core::Mapping<TIn, TOut>::apply(0, sofa::core::VecCoordId::position(), sofa::core::ConstVecCoordId::position());
./applications/plugins/RigidScale/mapping/RigidScaleToAffineMultiMapping.inl:220:		const InVecCoord1& x1_const = this->stateIn1->read(sofa::core::ConstVecCoordId::position())->getValue();
./applications/plugins/RigidScale/mapping/RigidScaleToAffineMultiMapping.inl:262:		const InVecCoord1& x1_const = this->stateIn1->read(sofa::core::ConstVecCoordId::position())->getValue();
./applications/plugins/RigidScale/mapping/RigidScaleToAffineMultiMapping.inl:263:		const InVecCoord2& x2_const = this->stateIn2->read(sofa::core::ConstVecCoordId::position())->getValue();
./applications/plugins/RigidScale/mapping/RigidScaleToAffineMultiMapping.inl:264:		const OutVecCoord& xout_const = this->stateOut->read(sofa::core::ConstVecCoordId::position())->getValue();
./applications/plugins/RigidScale/mapping/RigidScaleToRigidMultiMapping.inl:228:		const InVecCoord1& x1_const = this->stateIn1->read(sofa::core::ConstVecCoordId::position())->getValue();
./applications/plugins/RigidScale/mapping/RigidScaleToRigidMultiMapping.inl:229:		const InVecCoord2& x2_const = this->stateIn2->read(sofa::core::ConstVecCoordId::position())->getValue();
./applications/plugins/RigidScale/mapping/RigidScaleToRigidMultiMapping.inl:230:		const OutVecCoord& xout_const = this->stateOut->read(sofa::core::ConstVecCoordId::position())->getValue();
./SofaKernel/modules/SofaImplicitOdeSolver/SofaImplicitOdeSolver_test/EulerImplicitSolverDynamic_test.cpp:167:            Coord p0=dofs.get()->read(sofa::core::ConstVecCoordId::position())->getValue()[0];
./SofaKernel/modules/SofaImplicitOdeSolver/SofaImplicitOdeSolver_test/SpringSolverDynamic_test.cpp:98:            Coord p0=dofs.get()->read(sofa::core::ConstVecCoordId::position())->getValue()[0];
./SofaKernel/modules/SofaExplicitOdeSolver/SofaExplicitOdeSolver_test/EulerExplicitSolverDynamic_test.cpp:156:            Coord p0=dofs.get()->read(sofa::core::ConstVecCoordId::position())->getValue()[0];
./SofaKernel/modules/SofaMeshCollision/PointModel.inl:437:    const typename DataTypes::VecCoord& x = (*this->model->mstate->read(sofa::core::ConstVecCoordId::position())->getValue());



// this based on Hugo's advice
auto head_ctx = dome_head->getContext();
sofa::component::engine::NearestPointROI<Vec3Types>* nearPoint;
head_ctx->get(nearPoint);


  // std::cout << "j: " << j << "\n";
  // std::cout << "x[ta[j]]: " << x[ta[j]] << "\n";
  // std::cout << "x[ta[j]][0]: " << x[ta[j]][0] << "\n";
  // std::cout << "SQ(x[ta[j]][0]): " << SQ(x[ta[j]][0]) << ", " << \
  //             SQ(x[ta[j]][1]) <<  ", " << SQ(x[ta[j]][2]) << "\n";
  // // auto m_r = SQ(x[ta[j]][0]) + SQ(x[ta[j]][1]) + SQ(x[ta[j]][2]);
  // std::cout << "m_r: " << std::sqrt(SQ(x[ta[j]][0]) + SQ(x[ta[j]][1]) + SQ(x[ta[j]][2])) << "\n";
  // std::cout << " tetInfo->m_sPolarVecEul[j].m_r " << tetInfo->m_sPolarVecEul[j].m_r ;
  // std::cout << "tetInfo->m_sPolarVecEul[j].m_r: " << tetInfo->m_sPolarVecEul[j].m_r << "\n";
  // std::cout << "tetInfo->m_sPolarVecEul[j].m_theta: " << tetInfo->m_sPolarVecEul[j].m_theta << "\n";

  // sv=tetInfo->m_shapeVector[1];
  // std::cout << "x0: " << x0 << "\n";
  // std::cout << "ta: " << ta << "\n";
