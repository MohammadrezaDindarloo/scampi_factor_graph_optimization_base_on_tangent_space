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
 * Symbolic function: IK_residual_func_cost3_wrt_fh1_Nl13
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost3WrtFh1Nl13(
    const Scalar fh1, const Scalar fv1, const sym::Rot3<Scalar>& DeltaRot,
    const Eigen::Matrix<Scalar, 3, 1>& position_vector, const sym::Rot3<Scalar>& Rot_init,
    const Scalar epsilon) {
  // Total ops: 300

  // Unused inputs
  (void)epsilon;

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _DeltaRot = DeltaRot.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _Rot_init = Rot_init.Data();

  // Intermediate terms (110)
  const Scalar _tmp0 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp1 = _DeltaRot[0] * _Rot_init[3] - _DeltaRot[1] * _Rot_init[2] +
                       _DeltaRot[2] * _Rot_init[1] + _DeltaRot[3] * _Rot_init[0];
  const Scalar _tmp2 = _DeltaRot[0] * _Rot_init[2] + _DeltaRot[1] * _Rot_init[3] -
                       _DeltaRot[2] * _Rot_init[0] + _DeltaRot[3] * _Rot_init[1];
  const Scalar _tmp3 = 2 * _tmp2;
  const Scalar _tmp4 = _tmp1 * _tmp3;
  const Scalar _tmp5 = -_DeltaRot[0] * _Rot_init[0] - _DeltaRot[1] * _Rot_init[1] -
                       _DeltaRot[2] * _Rot_init[2] + _DeltaRot[3] * _Rot_init[3];
  const Scalar _tmp6 = -_DeltaRot[0] * _Rot_init[1] + _DeltaRot[1] * _Rot_init[0] +
                       _DeltaRot[2] * _Rot_init[3] + _DeltaRot[3] * _Rot_init[2];
  const Scalar _tmp7 = 2 * _tmp6;
  const Scalar _tmp8 = _tmp5 * _tmp7;
  const Scalar _tmp9 = Scalar(0.20999999999999999) * _tmp4 + Scalar(0.20999999999999999) * _tmp8;
  const Scalar _tmp10 = -_tmp9;
  const Scalar _tmp11 = -2 * std::pow(_tmp1, Scalar(2));
  const Scalar _tmp12 = -2 * std::pow(_tmp6, Scalar(2));
  const Scalar _tmp13 = Scalar(0.20999999999999999) * _tmp11 +
                        Scalar(0.20999999999999999) * _tmp12 + Scalar(0.20999999999999999);
  const Scalar _tmp14 = _tmp2 * _tmp7;
  const Scalar _tmp15 = 2 * _tmp1 * _tmp5;
  const Scalar _tmp16 =
      -Scalar(0.010999999999999999) * _tmp14 + Scalar(0.010999999999999999) * _tmp15;
  const Scalar _tmp17 = -_tmp13 + _tmp16;
  const Scalar _tmp18 = _tmp10 + _tmp17;
  const Scalar _tmp19 = Scalar(1.0) * _tmp18;
  const Scalar _tmp20 = _tmp13 + _tmp16;
  const Scalar _tmp21 = _tmp10 + _tmp20;
  const Scalar _tmp22 = Scalar(0.20999999999999999) * _tmp4 - Scalar(0.20999999999999999) * _tmp8;
  const Scalar _tmp23 = 1 - 2 * std::pow(_tmp2, Scalar(2));
  const Scalar _tmp24 = Scalar(0.20999999999999999) * _tmp12 + Scalar(0.20999999999999999) * _tmp23;
  const Scalar _tmp25 = _tmp1 * _tmp7;
  const Scalar _tmp26 = _tmp3 * _tmp5;
  const Scalar _tmp27 =
      -Scalar(0.010999999999999999) * _tmp25 - Scalar(0.010999999999999999) * _tmp26;
  const Scalar _tmp28 = -_tmp24 + _tmp27;
  const Scalar _tmp29 = _tmp22 + _tmp28;
  const Scalar _tmp30 = -_tmp22;
  const Scalar _tmp31 = _tmp28 + _tmp30;
  const Scalar _tmp32 = Scalar(1.0) * _tmp31;
  const Scalar _tmp33 = (-_tmp29 + _tmp32) / (-_tmp19 + _tmp21);
  const Scalar _tmp34 = _tmp19 * _tmp33 + _tmp32;
  const Scalar _tmp35 = Scalar(0.20999999999999999) * _tmp14 + Scalar(0.20999999999999999) * _tmp15;
  const Scalar _tmp36 = -_tmp35;
  const Scalar _tmp37 =
      -Scalar(0.010999999999999999) * _tmp11 - Scalar(0.010999999999999999) * _tmp23;
  const Scalar _tmp38 = Scalar(0.20999999999999999) * _tmp25 - Scalar(0.20999999999999999) * _tmp26;
  const Scalar _tmp39 = _tmp37 - _tmp38;
  const Scalar _tmp40 = _tmp36 + _tmp39;
  const Scalar _tmp41 = _tmp18 + position_vector(1, 0) + Scalar(8.3196563700000006);
  const Scalar _tmp42 = _tmp31 + position_vector(0, 0) + Scalar(1.9874742000000001);
  const Scalar _tmp43 = Scalar(1.0) / (_tmp42);
  const Scalar _tmp44 = _tmp41 * _tmp43;
  const Scalar _tmp45 = _tmp40 * _tmp44;
  const Scalar _tmp46 = _tmp17 + _tmp9;
  const Scalar _tmp47 = _tmp46 + position_vector(1, 0) + Scalar(8.3888750099999996);
  const Scalar _tmp48 = _tmp24 + _tmp27;
  const Scalar _tmp49 = _tmp30 + _tmp48;
  const Scalar _tmp50 = _tmp49 + position_vector(0, 0) + Scalar(-2.5202214700000001);
  const Scalar _tmp51 = std::pow(Scalar(std::pow(_tmp47, Scalar(2)) + std::pow(_tmp50, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp52 = _tmp50 * _tmp51;
  const Scalar _tmp53 = _tmp36 + _tmp37 + _tmp38;
  const Scalar _tmp54 = _tmp47 * _tmp51;
  const Scalar _tmp55 = _tmp35 + _tmp39;
  const Scalar _tmp56 = _tmp21 + position_vector(1, 0) + Scalar(-4.8333311099999996);
  const Scalar _tmp57 = _tmp29 + position_vector(0, 0) + Scalar(1.79662371);
  const Scalar _tmp58 = std::pow(Scalar(std::pow(_tmp56, Scalar(2)) + std::pow(_tmp57, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp59 = _tmp56 * _tmp58;
  const Scalar _tmp60 = _tmp57 * _tmp58;
  const Scalar _tmp61 = -_tmp45 * _tmp60 + _tmp55 * _tmp59;
  const Scalar _tmp62 = Scalar(1.0) / (_tmp44 * _tmp60 - _tmp59);
  const Scalar _tmp63 = _tmp44 * _tmp52 - _tmp54;
  const Scalar _tmp64 = _tmp62 * _tmp63;
  const Scalar _tmp65 = _tmp40 * _tmp60 - _tmp55 * _tmp60;
  const Scalar _tmp66 = -_tmp33 * (-_tmp45 * _tmp52 + _tmp53 * _tmp54 - _tmp61 * _tmp64) +
                        _tmp40 * _tmp52 - _tmp52 * _tmp53 - _tmp64 * _tmp65;
  const Scalar _tmp67 = Scalar(1.0) / (_tmp66);
  const Scalar _tmp68 = 0;
  const Scalar _tmp69 = _tmp60 * _tmp64;
  const Scalar _tmp70 =
      std::sqrt(Scalar(std::pow(_tmp41, Scalar(2)) + std::pow(_tmp42, Scalar(2))));
  const Scalar _tmp71 = _tmp43 * _tmp70;
  const Scalar _tmp72 = Scalar(1.0) * _tmp62;
  const Scalar _tmp73 = _tmp33 * _tmp61 * _tmp72 - _tmp65 * _tmp72;
  const Scalar _tmp74 = Scalar(1.0) / (_tmp70);
  const Scalar _tmp75 = _tmp71 * (-_tmp18 * _tmp42 * _tmp74 + _tmp31 * _tmp41 * _tmp74);
  const Scalar _tmp76 = _tmp21 * _tmp60 - _tmp29 * _tmp59 + _tmp60 * _tmp75;
  const Scalar _tmp77 = _tmp46 * _tmp52 - _tmp49 * _tmp54 + _tmp52 * _tmp75 - _tmp64 * _tmp76;
  const Scalar _tmp78 = _tmp67 * _tmp77;
  const Scalar _tmp79 = Scalar(1.0) / (_tmp77);
  const Scalar _tmp80 = _tmp66 * _tmp79;
  const Scalar _tmp81 = _tmp73 + _tmp80 * (-_tmp72 * _tmp76 - _tmp73 * _tmp78);
  const Scalar _tmp82 = _tmp67 * _tmp81;
  const Scalar _tmp83 = _tmp63 * _tmp67;
  const Scalar _tmp84 = -_tmp81 * _tmp83 + Scalar(1.0);
  const Scalar _tmp85 = _tmp60 * _tmp62;
  const Scalar _tmp86 = _tmp20 + _tmp9;
  const Scalar _tmp87 = _tmp86 + position_vector(1, 0) + Scalar(-4.7752063900000001);
  const Scalar _tmp88 = _tmp22 + _tmp48;
  const Scalar _tmp89 = _tmp88 + position_vector(0, 0) + Scalar(-2.71799795);
  const Scalar _tmp90 = std::pow(Scalar(std::pow(_tmp87, Scalar(2)) + std::pow(_tmp89, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp91 = _tmp87 * _tmp90;
  const Scalar _tmp92 = _tmp71 * _tmp91 * (_tmp52 * _tmp82 + _tmp84 * _tmp85);
  const Scalar _tmp93 = _tmp44 * _tmp62;
  const Scalar _tmp94 = -_tmp33 * (_tmp45 + _tmp61 * _tmp93) - _tmp40 + _tmp65 * _tmp93;
  const Scalar _tmp95 = _tmp80 * (-_tmp75 + _tmp76 * _tmp93 - _tmp78 * _tmp94) + _tmp94;
  const Scalar _tmp96 = _tmp67 * _tmp95;
  const Scalar _tmp97 = -_tmp44 - _tmp83 * _tmp95;
  const Scalar _tmp98 = _tmp89 * _tmp90;
  const Scalar _tmp99 = _tmp71 * _tmp98 * (_tmp52 * _tmp96 + _tmp85 * _tmp97 + Scalar(1.0));
  const Scalar _tmp100 = -_tmp86 * _tmp98 + _tmp88 * _tmp91;
  const Scalar _tmp101 = Scalar(1.0) * _tmp79;
  const Scalar _tmp102 = _tmp100 * _tmp71 * (_tmp101 * _tmp52 - _tmp101 * _tmp69);
  const Scalar _tmp103 = _tmp100 * _tmp101;
  const Scalar _tmp104 = _tmp103 * fh1;
  const Scalar _tmp105 = _tmp62 * _tmp97 * _tmp98;
  const Scalar _tmp106 = _tmp62 * _tmp84 * _tmp91;
  const Scalar _tmp107 = _tmp0 * _tmp68;
  const Scalar _tmp108 = _tmp96 * _tmp98;
  const Scalar _tmp109 = _tmp82 * _tmp91;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = -std::exp(-fh1);
  _res(1, 0) = -(-_tmp102 - _tmp92 - _tmp99) *
               std::exp(_tmp0 * _tmp71 * (_tmp52 * _tmp68 - _tmp68 * _tmp69) + _tmp102 * fh1 +
                        _tmp92 * fh1 + _tmp99 * fh1);
  _res(2, 0) = -(-_tmp103 * _tmp64 + _tmp105 + _tmp106) *
               std::exp(_tmp104 * _tmp64 - _tmp105 * fh1 - _tmp106 * fh1 + _tmp107 * _tmp64);
  _res(3, 0) =
      -(_tmp103 + _tmp108 + _tmp109) * std::exp(-_tmp104 - _tmp107 - _tmp108 * fh1 - _tmp109 * fh1);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
