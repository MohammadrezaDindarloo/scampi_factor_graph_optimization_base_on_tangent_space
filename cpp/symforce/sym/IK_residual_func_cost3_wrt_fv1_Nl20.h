// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     function/FUNCTION.h.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>

#include <sym/rot3.h>

namespace sym {

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: IK_residual_func_cost3_wrt_fv1_Nl20
 *
 * Args:
 *     fh1: Scalar
 *     fv1: Scalar
 *     DeltaRot: Rot3
 *     position_vector: Matrix31
 *     Rot_init: Rot3
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix41
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost3WrtFv1Nl20(
    const Scalar fh1, const Scalar fv1, const sym::Rot3<Scalar>& DeltaRot,
    const Eigen::Matrix<Scalar, 3, 1>& position_vector, const sym::Rot3<Scalar>& Rot_init,
    const Scalar epsilon) {
  // Total ops: 290

  // Unused inputs
  (void)epsilon;

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _DeltaRot = DeltaRot.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _Rot_init = Rot_init.Data();

  // Intermediate terms (103)
  const Scalar _tmp0 = -_DeltaRot[0] * _Rot_init[1] + _DeltaRot[1] * _Rot_init[0] +
                       _DeltaRot[2] * _Rot_init[3] + _DeltaRot[3] * _Rot_init[2];
  const Scalar _tmp1 = -2 * std::pow(_tmp0, Scalar(2));
  const Scalar _tmp2 = _DeltaRot[0] * _Rot_init[2] + _DeltaRot[1] * _Rot_init[3] -
                       _DeltaRot[2] * _Rot_init[0] + _DeltaRot[3] * _Rot_init[1];
  const Scalar _tmp3 = 1 - 2 * std::pow(_tmp2, Scalar(2));
  const Scalar _tmp4 = Scalar(0.20999999999999999) * _tmp1 + Scalar(0.20999999999999999) * _tmp3;
  const Scalar _tmp5 = -_tmp4;
  const Scalar _tmp6 = _DeltaRot[0] * _Rot_init[3] - _DeltaRot[1] * _Rot_init[2] +
                       _DeltaRot[2] * _Rot_init[1] + _DeltaRot[3] * _Rot_init[0];
  const Scalar _tmp7 = 2 * _tmp0 * _tmp6;
  const Scalar _tmp8 = -2 * _DeltaRot[0] * _Rot_init[0] - 2 * _DeltaRot[1] * _Rot_init[1] -
                       2 * _DeltaRot[2] * _Rot_init[2] + 2 * _DeltaRot[3] * _Rot_init[3];
  const Scalar _tmp9 = _tmp2 * _tmp8;
  const Scalar _tmp10 =
      -Scalar(0.010999999999999999) * _tmp7 - Scalar(0.010999999999999999) * _tmp9;
  const Scalar _tmp11 = 2 * _tmp2;
  const Scalar _tmp12 = _tmp11 * _tmp6;
  const Scalar _tmp13 = _tmp0 * _tmp8;
  const Scalar _tmp14 = Scalar(0.20999999999999999) * _tmp12 - Scalar(0.20999999999999999) * _tmp13;
  const Scalar _tmp15 = _tmp10 - _tmp14;
  const Scalar _tmp16 = _tmp15 + _tmp5;
  const Scalar _tmp17 = _tmp16 + position_vector(0, 0) + Scalar(1.9874742000000001);
  const Scalar _tmp18 = Scalar(0.20999999999999999) * _tmp12 + Scalar(0.20999999999999999) * _tmp13;
  const Scalar _tmp19 = -_tmp18;
  const Scalar _tmp20 = -2 * std::pow(_tmp6, Scalar(2));
  const Scalar _tmp21 = Scalar(0.20999999999999999) * _tmp1 + Scalar(0.20999999999999999) * _tmp20 +
                        Scalar(0.20999999999999999);
  const Scalar _tmp22 = _tmp0 * _tmp11;
  const Scalar _tmp23 = _tmp6 * _tmp8;
  const Scalar _tmp24 =
      -Scalar(0.010999999999999999) * _tmp22 + Scalar(0.010999999999999999) * _tmp23;
  const Scalar _tmp25 = -_tmp21 + _tmp24;
  const Scalar _tmp26 = _tmp19 + _tmp25;
  const Scalar _tmp27 = _tmp26 + position_vector(1, 0) + Scalar(8.3196563700000006);
  const Scalar _tmp28 = std::pow(Scalar(std::pow(_tmp17, Scalar(2)) + std::pow(_tmp27, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp29 = _tmp17 * _tmp28;
  const Scalar _tmp30 = _tmp15 + _tmp4;
  const Scalar _tmp31 = _tmp30 + position_vector(0, 0) + Scalar(-2.5202214700000001);
  const Scalar _tmp32 = Scalar(1.0) / (_tmp31);
  const Scalar _tmp33 = _tmp18 + _tmp25;
  const Scalar _tmp34 = _tmp33 + position_vector(1, 0) + Scalar(8.3888750099999996);
  const Scalar _tmp35 = _tmp32 * _tmp34;
  const Scalar _tmp36 = _tmp21 + _tmp24;
  const Scalar _tmp37 = _tmp18 + _tmp36;
  const Scalar _tmp38 = _tmp37 + position_vector(1, 0) + Scalar(-4.7752063900000001);
  const Scalar _tmp39 = _tmp10 + _tmp14;
  const Scalar _tmp40 = _tmp39 + _tmp4;
  const Scalar _tmp41 = _tmp40 + position_vector(0, 0) + Scalar(-2.71799795);
  const Scalar _tmp42 = std::pow(Scalar(std::pow(_tmp38, Scalar(2)) + std::pow(_tmp41, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp43 = _tmp41 * _tmp42;
  const Scalar _tmp44 = _tmp38 * _tmp42;
  const Scalar _tmp45 = _tmp35 * _tmp43 - _tmp44;
  const Scalar _tmp46 = _tmp27 * _tmp28;
  const Scalar _tmp47 = Scalar(1.0) / (_tmp29 * _tmp35 - _tmp46);
  const Scalar _tmp48 = _tmp45 * _tmp47;
  const Scalar _tmp49 =
      std::sqrt(Scalar(std::pow(_tmp31, Scalar(2)) + std::pow(_tmp34, Scalar(2))));
  const Scalar _tmp50 = Scalar(1.0) / (_tmp49);
  const Scalar _tmp51 = _tmp32 * _tmp49;
  const Scalar _tmp52 = _tmp51 * (_tmp30 * _tmp34 * _tmp50 - _tmp31 * _tmp33 * _tmp50);
  const Scalar _tmp53 = _tmp47 * (-_tmp16 * _tmp46 + _tmp26 * _tmp29 + _tmp29 * _tmp52);
  const Scalar _tmp54 = _tmp37 * _tmp43 - _tmp40 * _tmp44 + _tmp43 * _tmp52 - _tmp45 * _tmp53;
  const Scalar _tmp55 = Scalar(1.0) / (_tmp54);
  const Scalar _tmp56 = Scalar(1.0) * _tmp55;
  const Scalar _tmp57 = _tmp39 + _tmp5;
  const Scalar _tmp58 = _tmp19 + _tmp36;
  const Scalar _tmp59 = _tmp58 + position_vector(1, 0) + Scalar(-4.8333311099999996);
  const Scalar _tmp60 = _tmp57 + position_vector(0, 0) + Scalar(1.79662371);
  const Scalar _tmp61 = std::pow(Scalar(std::pow(_tmp59, Scalar(2)) + std::pow(_tmp60, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp62 = _tmp59 * _tmp61;
  const Scalar _tmp63 = _tmp60 * _tmp61;
  const Scalar _tmp64 = fh1 * (_tmp57 * _tmp62 - _tmp58 * _tmp63);
  const Scalar _tmp65 = Scalar(0.20999999999999999) * _tmp7 - Scalar(0.20999999999999999) * _tmp9;
  const Scalar _tmp66 =
      -Scalar(0.010999999999999999) * _tmp20 - Scalar(0.010999999999999999) * _tmp3;
  const Scalar _tmp67 = Scalar(0.20999999999999999) * _tmp22 + Scalar(0.20999999999999999) * _tmp23;
  const Scalar _tmp68 = _tmp66 - _tmp67;
  const Scalar _tmp69 = _tmp65 + _tmp68;
  const Scalar _tmp70 = -_tmp65 + _tmp68;
  const Scalar _tmp71 = _tmp29 * _tmp69 - _tmp29 * _tmp70;
  const Scalar _tmp72 = _tmp47 * _tmp71;
  const Scalar _tmp73 = Scalar(1.0) * _tmp33;
  const Scalar _tmp74 = Scalar(1.0) * _tmp30;
  const Scalar _tmp75 = (-_tmp16 + _tmp74) / (_tmp26 - _tmp73);
  const Scalar _tmp76 = _tmp35 * _tmp69;
  const Scalar _tmp77 = -_tmp29 * _tmp76 + _tmp46 * _tmp70;
  const Scalar _tmp78 = _tmp47 * _tmp77;
  const Scalar _tmp79 = -Scalar(1.0) * _tmp72 + Scalar(1.0) * _tmp75 * _tmp78;
  const Scalar _tmp80 = _tmp65 + _tmp66 + _tmp67;
  const Scalar _tmp81 = _tmp43 * _tmp69 - _tmp43 * _tmp80 - _tmp48 * _tmp71 -
                        _tmp75 * (-_tmp43 * _tmp76 + _tmp44 * _tmp80 - _tmp48 * _tmp77);
  const Scalar _tmp82 = Scalar(1.0) / (_tmp81);
  const Scalar _tmp83 = _tmp54 * _tmp82;
  const Scalar _tmp84 = _tmp55 * _tmp81;
  const Scalar _tmp85 = _tmp79 + _tmp84 * (-Scalar(1.0) * _tmp53 - _tmp79 * _tmp83);
  const Scalar _tmp86 = _tmp43 * _tmp82;
  const Scalar _tmp87 = _tmp45 * _tmp82;
  const Scalar _tmp88 = -_tmp85 * _tmp87 + Scalar(1.0);
  const Scalar _tmp89 = _tmp29 * _tmp47;
  const Scalar _tmp90 = _tmp62 * fh1;
  const Scalar _tmp91 = _tmp35 * _tmp72 - _tmp69 - _tmp75 * (_tmp35 * _tmp78 + _tmp76);
  const Scalar _tmp92 = _tmp84 * (_tmp35 * _tmp53 - _tmp52 - _tmp83 * _tmp91) + _tmp91;
  const Scalar _tmp93 = -_tmp35 - _tmp87 * _tmp92;
  const Scalar _tmp94 = _tmp63 * fh1;
  const Scalar _tmp95 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp96 = _tmp73 * _tmp75 + _tmp74;
  const Scalar _tmp97 = 0;
  const Scalar _tmp98 = _tmp82 * _tmp97;
  const Scalar _tmp99 = _tmp87 * _tmp97;
  const Scalar _tmp100 = _tmp51 * (_tmp43 * _tmp98 - _tmp89 * _tmp99);
  const Scalar _tmp101 = _tmp56 * _tmp64;
  const Scalar _tmp102 = _tmp47 * _tmp99;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = 0;
  _res(1, 0) =
      -_tmp100 *
      std::exp(_tmp100 * _tmp95 + _tmp51 * _tmp64 * (-_tmp29 * _tmp48 * _tmp56 + _tmp43 * _tmp56) +
               _tmp51 * _tmp90 * (_tmp85 * _tmp86 + _tmp88 * _tmp89) +
               _tmp51 * _tmp94 * (_tmp86 * _tmp92 + _tmp89 * _tmp93 + Scalar(1.0)));
  _res(2, 0) = -_tmp102 * std::exp(_tmp101 * _tmp48 + _tmp102 * _tmp95 - _tmp47 * _tmp88 * _tmp90 -
                                   _tmp47 * _tmp93 * _tmp94);
  _res(3, 0) = _tmp98 * std::exp(-_tmp101 - _tmp82 * _tmp85 * _tmp90 - _tmp82 * _tmp92 * _tmp94 -
                                 _tmp95 * _tmp98);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
