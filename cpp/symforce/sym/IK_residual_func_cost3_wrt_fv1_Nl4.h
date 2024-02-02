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
 * Symbolic function: IK_residual_func_cost3_wrt_fv1_Nl4
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost3WrtFv1Nl4(
    const Scalar fh1, const Scalar fv1, const sym::Rot3<Scalar>& DeltaRot,
    const Eigen::Matrix<Scalar, 3, 1>& position_vector, const sym::Rot3<Scalar>& Rot_init,
    const Scalar epsilon) {
  // Total ops: 284

  // Unused inputs
  (void)epsilon;

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _DeltaRot = DeltaRot.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _Rot_init = Rot_init.Data();

  // Intermediate terms (104)
  const Scalar _tmp0 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp1 = _DeltaRot[0] * _Rot_init[2] + _DeltaRot[1] * _Rot_init[3] -
                       _DeltaRot[2] * _Rot_init[0] + _DeltaRot[3] * _Rot_init[1];
  const Scalar _tmp2 = _DeltaRot[0] * _Rot_init[3] - _DeltaRot[1] * _Rot_init[2] +
                       _DeltaRot[2] * _Rot_init[1] + _DeltaRot[3] * _Rot_init[0];
  const Scalar _tmp3 = 2 * _tmp2;
  const Scalar _tmp4 = _tmp1 * _tmp3;
  const Scalar _tmp5 = -_DeltaRot[0] * _Rot_init[0] - _DeltaRot[1] * _Rot_init[1] -
                       _DeltaRot[2] * _Rot_init[2] + _DeltaRot[3] * _Rot_init[3];
  const Scalar _tmp6 = -_DeltaRot[0] * _Rot_init[1] + _DeltaRot[1] * _Rot_init[0] +
                       _DeltaRot[2] * _Rot_init[3] + _DeltaRot[3] * _Rot_init[2];
  const Scalar _tmp7 = 2 * _tmp6;
  const Scalar _tmp8 = _tmp5 * _tmp7;
  const Scalar _tmp9 = Scalar(0.20999999999999999) * _tmp4 + Scalar(0.20999999999999999) * _tmp8;
  const Scalar _tmp10 = -2 * std::pow(_tmp2, Scalar(2));
  const Scalar _tmp11 = 1 - 2 * std::pow(_tmp6, Scalar(2));
  const Scalar _tmp12 = Scalar(0.20999999999999999) * _tmp10 + Scalar(0.20999999999999999) * _tmp11;
  const Scalar _tmp13 = _tmp1 * _tmp7;
  const Scalar _tmp14 = _tmp3 * _tmp5;
  const Scalar _tmp15 =
      -Scalar(0.010999999999999999) * _tmp13 + Scalar(0.010999999999999999) * _tmp14;
  const Scalar _tmp16 = -_tmp12 + _tmp15;
  const Scalar _tmp17 = _tmp16 + _tmp9;
  const Scalar _tmp18 = _tmp17 + position_vector(1, 0) + Scalar(8.3888750099999996);
  const Scalar _tmp19 = Scalar(0.20999999999999999) * _tmp4 - Scalar(0.20999999999999999) * _tmp8;
  const Scalar _tmp20 = -_tmp19;
  const Scalar _tmp21 = _tmp2 * _tmp7;
  const Scalar _tmp22 = 2 * _tmp1 * _tmp5;
  const Scalar _tmp23 =
      -Scalar(0.010999999999999999) * _tmp21 - Scalar(0.010999999999999999) * _tmp22;
  const Scalar _tmp24 = -2 * std::pow(_tmp1, Scalar(2));
  const Scalar _tmp25 = Scalar(0.20999999999999999) * _tmp11 + Scalar(0.20999999999999999) * _tmp24;
  const Scalar _tmp26 = _tmp23 + _tmp25;
  const Scalar _tmp27 = _tmp20 + _tmp26;
  const Scalar _tmp28 = _tmp27 + position_vector(0, 0) + Scalar(-2.5202214700000001);
  const Scalar _tmp29 = std::pow(Scalar(std::pow(_tmp18, Scalar(2)) + std::pow(_tmp28, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp30 = _tmp28 * _tmp29;
  const Scalar _tmp31 = _tmp12 + _tmp15;
  const Scalar _tmp32 = _tmp31 + _tmp9;
  const Scalar _tmp33 = _tmp32 + position_vector(1, 0) + Scalar(-4.7752063900000001);
  const Scalar _tmp34 = _tmp19 + _tmp26;
  const Scalar _tmp35 = _tmp34 + position_vector(0, 0) + Scalar(-2.71799795);
  const Scalar _tmp36 = std::pow(Scalar(std::pow(_tmp33, Scalar(2)) + std::pow(_tmp35, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp37 = _tmp33 * _tmp36;
  const Scalar _tmp38 = -_tmp9;
  const Scalar _tmp39 = _tmp31 + _tmp38;
  const Scalar _tmp40 = _tmp39 + position_vector(1, 0) + Scalar(-4.8333311099999996);
  const Scalar _tmp41 = _tmp23 - _tmp25;
  const Scalar _tmp42 = _tmp19 + _tmp41;
  const Scalar _tmp43 = _tmp42 + position_vector(0, 0) + Scalar(1.79662371);
  const Scalar _tmp44 = Scalar(1.0) / (_tmp43);
  const Scalar _tmp45 = _tmp40 * _tmp44;
  const Scalar _tmp46 = _tmp35 * _tmp36;
  const Scalar _tmp47 = -_tmp37 + _tmp45 * _tmp46;
  const Scalar _tmp48 = _tmp18 * _tmp29;
  const Scalar _tmp49 = Scalar(1.0) / (_tmp30 * _tmp45 - _tmp48);
  const Scalar _tmp50 = _tmp47 * _tmp49;
  const Scalar _tmp51 = Scalar(0.20999999999999999) * _tmp21 - Scalar(0.20999999999999999) * _tmp22;
  const Scalar _tmp52 = -Scalar(0.010999999999999999) * _tmp10 -
                        Scalar(0.010999999999999999) * _tmp24 + Scalar(-0.010999999999999999);
  const Scalar _tmp53 = Scalar(0.20999999999999999) * _tmp13 + Scalar(0.20999999999999999) * _tmp14;
  const Scalar _tmp54 = _tmp52 + _tmp53;
  const Scalar _tmp55 = -_tmp51 + _tmp54;
  const Scalar _tmp56 = _tmp46 * _tmp55;
  const Scalar _tmp57 = _tmp30 * _tmp55;
  const Scalar _tmp58 = _tmp51 + _tmp52 - _tmp53;
  const Scalar _tmp59 = -_tmp45 * _tmp57 + _tmp48 * _tmp58;
  const Scalar _tmp60 = _tmp51 + _tmp54;
  const Scalar _tmp61 = Scalar(1.0) * _tmp42;
  const Scalar _tmp62 = Scalar(1.0) * _tmp39;
  const Scalar _tmp63 = (-_tmp27 + _tmp61) / (_tmp17 - _tmp62);
  const Scalar _tmp64 = -_tmp30 * _tmp58 + _tmp57;
  const Scalar _tmp65 = -_tmp46 * _tmp60 - _tmp50 * _tmp64 + _tmp56 -
                        _tmp63 * (_tmp37 * _tmp60 - _tmp45 * _tmp56 - _tmp50 * _tmp59);
  const Scalar _tmp66 = Scalar(1.0) / (_tmp65);
  const Scalar _tmp67 = _tmp61 + _tmp62 * _tmp63;
  const Scalar _tmp68 = 0;
  const Scalar _tmp69 = _tmp50 * _tmp68;
  const Scalar _tmp70 =
      std::sqrt(Scalar(std::pow(_tmp40, Scalar(2)) + std::pow(_tmp43, Scalar(2))));
  const Scalar _tmp71 = _tmp44 * _tmp70;
  const Scalar _tmp72 = _tmp71 * (-_tmp30 * _tmp69 + _tmp46 * _tmp68);
  const Scalar _tmp73 = Scalar(1.0) * _tmp49;
  const Scalar _tmp74 = _tmp59 * _tmp63 * _tmp73 - _tmp64 * _tmp73;
  const Scalar _tmp75 = Scalar(1.0) / (_tmp70);
  const Scalar _tmp76 = _tmp71 * (-_tmp39 * _tmp43 * _tmp75 + _tmp40 * _tmp42 * _tmp75);
  const Scalar _tmp77 = _tmp17 * _tmp30 - _tmp27 * _tmp48 + _tmp30 * _tmp76;
  const Scalar _tmp78 = _tmp32 * _tmp46 - _tmp34 * _tmp37 + _tmp46 * _tmp76 - _tmp50 * _tmp77;
  const Scalar _tmp79 = _tmp66 * _tmp78;
  const Scalar _tmp80 = Scalar(1.0) / (_tmp78);
  const Scalar _tmp81 = _tmp65 * _tmp80;
  const Scalar _tmp82 = _tmp74 + _tmp81 * (-_tmp73 * _tmp77 - _tmp74 * _tmp79);
  const Scalar _tmp83 = _tmp47 * _tmp66;
  const Scalar _tmp84 = -_tmp82 * _tmp83 + Scalar(1.0);
  const Scalar _tmp85 = _tmp30 * _tmp49;
  const Scalar _tmp86 = _tmp46 * _tmp66;
  const Scalar _tmp87 = _tmp16 + _tmp38;
  const Scalar _tmp88 = _tmp87 + position_vector(1, 0) + Scalar(8.3196563700000006);
  const Scalar _tmp89 = _tmp20 + _tmp41;
  const Scalar _tmp90 = _tmp89 + position_vector(0, 0) + Scalar(1.9874742000000001);
  const Scalar _tmp91 = std::pow(Scalar(std::pow(_tmp88, Scalar(2)) + std::pow(_tmp90, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp92 = _tmp88 * _tmp91;
  const Scalar _tmp93 = _tmp92 * fh1;
  const Scalar _tmp94 = Scalar(1.0) * _tmp80;
  const Scalar _tmp95 = _tmp90 * _tmp91;
  const Scalar _tmp96 = fh1 * (-_tmp87 * _tmp95 + _tmp89 * _tmp92);
  const Scalar _tmp97 = _tmp45 * _tmp49;
  const Scalar _tmp98 = -_tmp55 - _tmp63 * (_tmp45 * _tmp55 + _tmp59 * _tmp97) + _tmp64 * _tmp97;
  const Scalar _tmp99 = _tmp81 * (-_tmp76 + _tmp77 * _tmp97 - _tmp79 * _tmp98) + _tmp98;
  const Scalar _tmp100 = -_tmp45 - _tmp83 * _tmp99;
  const Scalar _tmp101 = _tmp95 * fh1;
  const Scalar _tmp102 = _tmp94 * _tmp96;
  const Scalar _tmp103 = _tmp0 * _tmp68;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = 0;
  _res(1, 0) =
      -_tmp72 * std::exp(_tmp0 * _tmp72 +
                         _tmp101 * _tmp71 * (_tmp100 * _tmp85 + _tmp86 * _tmp99 + Scalar(1.0)) +
                         _tmp71 * _tmp93 * (_tmp82 * _tmp86 + _tmp84 * _tmp85) +
                         _tmp71 * _tmp96 * (-_tmp30 * _tmp50 * _tmp94 + _tmp46 * _tmp94));
  _res(2, 0) = -_tmp69 * std::exp(-_tmp100 * _tmp101 * _tmp49 + _tmp102 * _tmp50 +
                                  _tmp103 * _tmp50 - _tmp49 * _tmp84 * _tmp93);
  _res(3, 0) =
      _tmp68 * std::exp(-_tmp101 * _tmp66 * _tmp99 - _tmp102 - _tmp103 - _tmp66 * _tmp82 * _tmp93);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym