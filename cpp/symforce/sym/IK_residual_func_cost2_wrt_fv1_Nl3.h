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
 * Symbolic function: IK_residual_func_cost2_wrt_fv1_Nl3
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost2WrtFv1Nl3(
    const Scalar fh1, const Scalar fv1, const sym::Rot3<Scalar>& DeltaRot,
    const Eigen::Matrix<Scalar, 3, 1>& position_vector, const sym::Rot3<Scalar>& Rot_init,
    const Scalar epsilon) {
  // Total ops: 593

  // Unused inputs
  (void)epsilon;

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _DeltaRot = DeltaRot.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _Rot_init = Rot_init.Data();

  // Intermediate terms (188)
  const Scalar _tmp0 = Scalar(1.0) / (fh1);
  const Scalar _tmp1 = std::asinh(_tmp0 * fv1);
  const Scalar _tmp2 = Scalar(1.0) * _tmp0 /
                       std::sqrt(Scalar(1 + std::pow(fv1, Scalar(2)) / std::pow(fh1, Scalar(2))));
  const Scalar _tmp3 = _DeltaRot[0] * _Rot_init[3] - _DeltaRot[1] * _Rot_init[2] +
                       _DeltaRot[2] * _Rot_init[1] + _DeltaRot[3] * _Rot_init[0];
  const Scalar _tmp4 = -2 * std::pow(_tmp3, Scalar(2));
  const Scalar _tmp5 = -_DeltaRot[0] * _Rot_init[1] + _DeltaRot[1] * _Rot_init[0] +
                       _DeltaRot[2] * _Rot_init[3] + _DeltaRot[3] * _Rot_init[2];
  const Scalar _tmp6 = 1 - 2 * std::pow(_tmp5, Scalar(2));
  const Scalar _tmp7 = Scalar(0.20999999999999999) * _tmp4 + Scalar(0.20999999999999999) * _tmp6;
  const Scalar _tmp8 = -_tmp7;
  const Scalar _tmp9 = _DeltaRot[0] * _Rot_init[2] + _DeltaRot[1] * _Rot_init[3] -
                       _DeltaRot[2] * _Rot_init[0] + _DeltaRot[3] * _Rot_init[1];
  const Scalar _tmp10 = 2 * _tmp9;
  const Scalar _tmp11 = _tmp10 * _tmp5;
  const Scalar _tmp12 = -2 * _DeltaRot[0] * _Rot_init[0] - 2 * _DeltaRot[1] * _Rot_init[1] -
                        2 * _DeltaRot[2] * _Rot_init[2] + 2 * _DeltaRot[3] * _Rot_init[3];
  const Scalar _tmp13 = _tmp12 * _tmp3;
  const Scalar _tmp14 = _tmp11 - _tmp13;
  const Scalar _tmp15 = Scalar(0.010999999999999999) * _tmp14;
  const Scalar _tmp16 = -_tmp15;
  const Scalar _tmp17 = _tmp10 * _tmp3;
  const Scalar _tmp18 = _tmp12 * _tmp5;
  const Scalar _tmp19 = Scalar(0.20999999999999999) * _tmp17 + Scalar(0.20999999999999999) * _tmp18;
  const Scalar _tmp20 = _tmp16 - _tmp19;
  const Scalar _tmp21 = _tmp20 + _tmp8;
  const Scalar _tmp22 = _tmp21 + position_vector(1, 0);
  const Scalar _tmp23 = Scalar(0.20999999999999999) * _tmp17 - Scalar(0.20999999999999999) * _tmp18;
  const Scalar _tmp24 = -_tmp23;
  const Scalar _tmp25 = -2 * std::pow(_tmp9, Scalar(2));
  const Scalar _tmp26 = Scalar(0.20999999999999999) * _tmp25 + Scalar(0.20999999999999999) * _tmp6;
  const Scalar _tmp27 = 2 * _tmp3 * _tmp5;
  const Scalar _tmp28 = _tmp12 * _tmp9;
  const Scalar _tmp29 = _tmp27 + _tmp28;
  const Scalar _tmp30 = -Scalar(0.010999999999999999) * _tmp29;
  const Scalar _tmp31 = -_tmp26 + _tmp30;
  const Scalar _tmp32 = _tmp24 + _tmp31;
  const Scalar _tmp33 = _tmp32 + position_vector(0, 0);
  const Scalar _tmp34 = Scalar(9.6622558468725703) * fh1;
  const Scalar _tmp35 = _tmp26 + _tmp30;
  const Scalar _tmp36 = _tmp24 + _tmp35;
  const Scalar _tmp37 = _tmp16 + _tmp19;
  const Scalar _tmp38 = _tmp37 + _tmp7;
  const Scalar _tmp39 = Scalar(1.0) * _tmp38;
  const Scalar _tmp40 = -_tmp39;
  const Scalar _tmp41 = _tmp37 + _tmp8;
  const Scalar _tmp42 = _tmp40 + _tmp41;
  const Scalar _tmp43 = _tmp20 + _tmp7;
  const Scalar _tmp44 = Scalar(1.0) / (_tmp40 + _tmp43);
  const Scalar _tmp45 = _tmp23 + _tmp31;
  const Scalar _tmp46 = _tmp23 + _tmp35;
  const Scalar _tmp47 = Scalar(1.0) * _tmp46;
  const Scalar _tmp48 = _tmp44 * (-_tmp45 + _tmp47);
  const Scalar _tmp49 = _tmp42 * _tmp48;
  const Scalar _tmp50 = Scalar(1.0) / (-_tmp36 + _tmp47 - _tmp49);
  const Scalar _tmp51 = Scalar(1.0) * _tmp50;
  const Scalar _tmp52 = Scalar(0.20999999999999999) * _tmp11 + Scalar(0.20999999999999999) * _tmp13;
  const Scalar _tmp53 = -Scalar(0.010999999999999999) * _tmp25 -
                        Scalar(0.010999999999999999) * _tmp4 + Scalar(-0.010999999999999999);
  const Scalar _tmp54 = Scalar(0.20999999999999999) * _tmp27 - Scalar(0.20999999999999999) * _tmp28;
  const Scalar _tmp55 = _tmp53 - _tmp54;
  const Scalar _tmp56 = _tmp52 + _tmp55;
  const Scalar _tmp57 = _tmp43 + position_vector(1, 0);
  const Scalar _tmp58 = _tmp57 + Scalar(-4.8333311099999996);
  const Scalar _tmp59 = _tmp45 + position_vector(0, 0);
  const Scalar _tmp60 = _tmp59 + Scalar(1.79662371);
  const Scalar _tmp61 = std::pow(Scalar(std::pow(_tmp58, Scalar(2)) + std::pow(_tmp60, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp62 = _tmp58 * _tmp61;
  const Scalar _tmp63 = _tmp38 + position_vector(1, 0);
  const Scalar _tmp64 = _tmp63 + Scalar(-4.7752063900000001);
  const Scalar _tmp65 = _tmp46 + position_vector(0, 0);
  const Scalar _tmp66 = _tmp65 + Scalar(-2.71799795);
  const Scalar _tmp67 = Scalar(1.0) / (_tmp66);
  const Scalar _tmp68 = _tmp64 * _tmp67;
  const Scalar _tmp69 = _tmp53 + _tmp54;
  const Scalar _tmp70 = _tmp52 + _tmp69;
  const Scalar _tmp71 = _tmp60 * _tmp61;
  const Scalar _tmp72 = _tmp70 * _tmp71;
  const Scalar _tmp73 = _tmp56 * _tmp62 - _tmp68 * _tmp72;
  const Scalar _tmp74 = Scalar(1.0) / (-_tmp62 + _tmp68 * _tmp71);
  const Scalar _tmp75 = Scalar(1.0) * _tmp74;
  const Scalar _tmp76 = _tmp73 * _tmp75;
  const Scalar _tmp77 = -_tmp56 * _tmp71 + _tmp72;
  const Scalar _tmp78 = _tmp48 * _tmp76 - _tmp75 * _tmp77;
  const Scalar _tmp79 = _tmp41 + position_vector(1, 0);
  const Scalar _tmp80 = _tmp79 + Scalar(8.3888750099999996);
  const Scalar _tmp81 = _tmp36 + position_vector(0, 0);
  const Scalar _tmp82 = _tmp81 + Scalar(-2.5202214700000001);
  const Scalar _tmp83 = std::pow(Scalar(std::pow(_tmp80, Scalar(2)) + std::pow(_tmp82, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp84 = _tmp82 * _tmp83;
  const Scalar _tmp85 = _tmp70 * _tmp84;
  const Scalar _tmp86 = -_tmp52;
  const Scalar _tmp87 = _tmp69 + _tmp86;
  const Scalar _tmp88 = _tmp80 * _tmp83;
  const Scalar _tmp89 = _tmp68 * _tmp84 - _tmp88;
  const Scalar _tmp90 = _tmp74 * _tmp89;
  const Scalar _tmp91 = -_tmp68 * _tmp85 - _tmp73 * _tmp90 + _tmp87 * _tmp88;
  const Scalar _tmp92 = -_tmp48 * _tmp91 - _tmp77 * _tmp90 - _tmp84 * _tmp87 + _tmp85;
  const Scalar _tmp93 = Scalar(1.0) / (_tmp92);
  const Scalar _tmp94 =
      std::sqrt(Scalar(std::pow(_tmp64, Scalar(2)) + std::pow(_tmp66, Scalar(2))));
  const Scalar _tmp95 = Scalar(1.0) / (_tmp94);
  const Scalar _tmp96 = _tmp67 * _tmp94;
  const Scalar _tmp97 = _tmp96 * (-_tmp38 * _tmp66 * _tmp95 + _tmp46 * _tmp64 * _tmp95);
  const Scalar _tmp98 = _tmp43 * _tmp71 - _tmp45 * _tmp62 + _tmp71 * _tmp97;
  const Scalar _tmp99 = -_tmp36 * _tmp88 + _tmp41 * _tmp84 + _tmp84 * _tmp97 - _tmp90 * _tmp98;
  const Scalar _tmp100 = _tmp93 * _tmp99;
  const Scalar _tmp101 = Scalar(1.0) / (_tmp99);
  const Scalar _tmp102 = _tmp101 * _tmp92;
  const Scalar _tmp103 = _tmp102 * (-_tmp100 * _tmp78 - _tmp75 * _tmp98);
  const Scalar _tmp104 = _tmp93 * (_tmp103 + _tmp78);
  const Scalar _tmp105 = _tmp42 * _tmp50;
  const Scalar _tmp106 = _tmp103 * _tmp105 - _tmp104 * _tmp91 - _tmp76;
  const Scalar _tmp107 = Scalar(1.0) * _tmp44;
  const Scalar _tmp108 = _tmp33 + Scalar(1.9874742000000001);
  const Scalar _tmp109 = _tmp22 + Scalar(8.3196563700000006);
  const Scalar _tmp110 =
      std::pow(Scalar(std::pow(_tmp108, Scalar(2)) + std::pow(_tmp109, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp111 = _tmp109 * _tmp110;
  const Scalar _tmp112 = _tmp111 * fh1;
  const Scalar _tmp113 = _tmp68 * _tmp74;
  const Scalar _tmp114 = _tmp113 * _tmp73 + _tmp68 * _tmp70;
  const Scalar _tmp115 = _tmp113 * _tmp77 - _tmp114 * _tmp48 - _tmp70;
  const Scalar _tmp116 = _tmp102 * (-_tmp100 * _tmp115 + _tmp113 * _tmp98 - _tmp97);
  const Scalar _tmp117 = _tmp93 * (_tmp115 + _tmp116);
  const Scalar _tmp118 = _tmp105 * _tmp116 + _tmp114 - _tmp117 * _tmp91;
  const Scalar _tmp119 = _tmp108 * _tmp110;
  const Scalar _tmp120 = _tmp119 * fh1;
  const Scalar _tmp121 = fh1 * (_tmp55 + _tmp86);
  const Scalar _tmp122 = -_tmp111 * _tmp121 - Scalar(5.1796800000000003) * _tmp14 - _tmp21 * fv1;
  const Scalar _tmp123 = _tmp48 * _tmp51;
  const Scalar _tmp124 = _tmp44 * (_tmp49 * _tmp51 + Scalar(1.0));
  const Scalar _tmp125 = Scalar(1.0) * _tmp123 - Scalar(1.0) * _tmp124;
  const Scalar _tmp126 = _tmp102 * _tmp51;
  const Scalar _tmp127 = Scalar(1.0) * _tmp101;
  const Scalar _tmp128 = _tmp126 * _tmp42 - _tmp127 * _tmp91;
  const Scalar _tmp129 = fh1 * (_tmp111 * _tmp32 - _tmp119 * _tmp21);
  const Scalar _tmp130 = _tmp119 * _tmp121 + Scalar(5.1796800000000003) * _tmp29 + _tmp32 * fv1;
  const Scalar _tmp131 = _tmp42 * _tmp44;
  const Scalar _tmp132 = Scalar(1.0) * _tmp131 * _tmp51 - Scalar(1.0) * _tmp51;
  const Scalar _tmp133 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp134 = _tmp39 * _tmp48 + _tmp47;
  const Scalar _tmp135 = 0;
  const Scalar _tmp136 = _tmp134 * _tmp50;
  const Scalar _tmp137 = _tmp44 * (-_tmp135 * _tmp91 - _tmp136 * _tmp42 + _tmp40);
  const Scalar _tmp138 = -Scalar(1.0) * _tmp136 - Scalar(1.0) * _tmp137 + Scalar(1.0);
  const Scalar _tmp139 = Scalar(1.0) * _tmp112 * (_tmp103 * _tmp51 - _tmp106 * _tmp107) +
                         Scalar(1.0) * _tmp120 * (-_tmp107 * _tmp118 + _tmp116 * _tmp51) +
                         _tmp122 * _tmp125 +
                         Scalar(1.0) * _tmp129 * (-_tmp107 * _tmp128 + _tmp126) +
                         _tmp130 * _tmp132 + _tmp133 * _tmp138;
  const Scalar _tmp140 = -_tmp104 * _tmp89 + Scalar(1.0);
  const Scalar _tmp141 = _tmp71 * _tmp74;
  const Scalar _tmp142 = _tmp135 * _tmp90;
  const Scalar _tmp143 = _tmp96 * (_tmp135 * _tmp84 - _tmp142 * _tmp71);
  const Scalar _tmp144 = -_tmp117 * _tmp89 - _tmp68;
  const Scalar _tmp145 = _tmp101 * _tmp75 * _tmp89;
  const Scalar _tmp146 = -_tmp112 * _tmp96 * (_tmp104 * _tmp84 + _tmp140 * _tmp141) -
                         _tmp120 * _tmp96 * (_tmp117 * _tmp84 + _tmp141 * _tmp144 + Scalar(1.0)) -
                         _tmp129 * _tmp96 * (_tmp127 * _tmp84 - _tmp145 * _tmp71) -
                         _tmp133 * _tmp143;
  const Scalar _tmp147 = Scalar(1.0) / (_tmp146);
  const Scalar _tmp148 = std::asinh(_tmp139 * _tmp147);
  const Scalar _tmp149 = Scalar(9.6622558468725703) * _tmp146;
  const Scalar _tmp150 =
      -_tmp148 * _tmp149 -
      Scalar(4.7752063900000001) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp63), Scalar(2)) +
                     Scalar(0.32397683292140877) *
                         std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp65), Scalar(2))));
  const Scalar _tmp151 = Scalar(0.1034955) * _tmp147;
  const Scalar _tmp152 = _tmp150 * _tmp151;
  const Scalar _tmp153 = Scalar(1.0) * _tmp148;
  const Scalar _tmp154 = Scalar(9.6622558468725703) * _tmp143;
  const Scalar _tmp155 = std::pow(_tmp146, Scalar(-2));
  const Scalar _tmp156 = _tmp143 * _tmp155;
  const Scalar _tmp157 = _tmp15 + _tmp19 + _tmp7;
  const Scalar _tmp158 =
      (-_tmp139 * _tmp156 + _tmp147 * (_tmp125 * _tmp157 + _tmp132 * _tmp32 - _tmp138)) /
      std::sqrt(Scalar(std::pow(_tmp139, Scalar(2)) * _tmp155 + 1));
  const Scalar _tmp159 = _tmp130 * _tmp51;
  const Scalar _tmp160 = _tmp106 * _tmp112 * _tmp44 + _tmp118 * _tmp120 * _tmp44 +
                         _tmp122 * _tmp124 + _tmp128 * _tmp129 * _tmp44 - _tmp131 * _tmp159 +
                         _tmp133 * _tmp137;
  const Scalar _tmp161 = _tmp133 * _tmp135;
  const Scalar _tmp162 = _tmp112 * _tmp140 * _tmp74 + _tmp120 * _tmp144 * _tmp74 -
                         _tmp129 * _tmp145 - _tmp161 * _tmp90;
  const Scalar _tmp163 = Scalar(1.0) / (_tmp162);
  const Scalar _tmp164 = std::asinh(_tmp160 * _tmp163);
  const Scalar _tmp165 = Scalar(9.6622558468725703) * _tmp162;
  const Scalar _tmp166 =
      -_tmp164 * _tmp165 -
      Scalar(4.8333311099999996) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp57), Scalar(2)) +
                     Scalar(0.13817235445745474) *
                         std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp59 - 1), Scalar(2))));
  const Scalar _tmp167 = Scalar(0.1034955) * _tmp163;
  const Scalar _tmp168 = _tmp166 * _tmp167;
  const Scalar _tmp169 = std::pow(_tmp162, Scalar(-2));
  const Scalar _tmp170 = _tmp142 * _tmp169;
  const Scalar _tmp171 = _tmp32 * _tmp51;
  const Scalar _tmp172 =
      (-_tmp160 * _tmp170 + _tmp163 * (_tmp124 * _tmp157 - _tmp131 * _tmp171 - _tmp137)) /
      std::sqrt(Scalar(std::pow(_tmp160, Scalar(2)) * _tmp169 + 1));
  const Scalar _tmp173 = Scalar(9.6622558468725703) * _tmp135;
  const Scalar _tmp174 = _tmp173 * _tmp90;
  const Scalar _tmp175 = Scalar(1.0) * _tmp164;
  const Scalar _tmp176 = -_tmp103 * _tmp112 * _tmp50 - _tmp116 * _tmp120 * _tmp50 -
                         _tmp122 * _tmp123 - _tmp126 * _tmp129 + _tmp133 * _tmp136 + _tmp159;
  const Scalar _tmp177 = _tmp104 * _tmp112 + _tmp117 * _tmp120 + _tmp127 * _tmp129 + _tmp161;
  const Scalar _tmp178 = Scalar(1.0) / (_tmp177);
  const Scalar _tmp179 = std::asinh(_tmp176 * _tmp178);
  const Scalar _tmp180 = Scalar(9.6622558468725703) * _tmp177;
  const Scalar _tmp181 =
      -_tmp179 * _tmp180 -
      Scalar(8.3888750099999996) *
          std::sqrt(
              Scalar(Scalar(0.090254729040973036) *
                         std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp81), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp79 - 1), Scalar(2))));
  const Scalar _tmp182 = Scalar(0.1034955) * _tmp178;
  const Scalar _tmp183 = _tmp181 * _tmp182;
  const Scalar _tmp184 = Scalar(1.0) * _tmp179;
  const Scalar _tmp185 = std::pow(_tmp177, Scalar(-2));
  const Scalar _tmp186 = _tmp135 * _tmp185;
  const Scalar _tmp187 = (_tmp176 * _tmp186 + _tmp178 * (-_tmp123 * _tmp157 - _tmp136 + _tmp171)) /
                         std::sqrt(Scalar(std::pow(_tmp176, Scalar(2)) * _tmp185 + 1));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      _tmp34 *
      (-_tmp2 * std::cosh(Scalar(1.0) * _tmp1) +
       _tmp2 * std::cosh(
                   Scalar(0.1034955) * _tmp0 *
                   (-_tmp1 * _tmp34 -
                    Scalar(8.3196563700000006) *
                        std::sqrt(Scalar(
                            std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp22 - 1), Scalar(2)) +
                            Scalar(0.057067943376852184) *
                                std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp33 - 1),
                                         Scalar(2)))))));
  _res(1, 0) = _tmp149 * (-Scalar(1.0) * _tmp158 * std::cosh(_tmp153) -
                          (-Scalar(0.1034955) * _tmp150 * _tmp156 +
                           _tmp151 * (-_tmp148 * _tmp154 - _tmp149 * _tmp158)) *
                              std::cosh(_tmp152)) +
               _tmp154 * (-std::sinh(_tmp152) - std::sinh(_tmp153));
  _res(2, 0) = _tmp165 * (-Scalar(1.0) * _tmp172 * std::cosh(_tmp175) -
                          (-Scalar(0.1034955) * _tmp166 * _tmp170 +
                           _tmp167 * (-_tmp164 * _tmp174 - _tmp165 * _tmp172)) *
                              std::cosh(_tmp168)) +
               _tmp174 * (-std::sinh(_tmp168) - std::sinh(_tmp175));
  _res(3, 0) = -_tmp173 * (-std::sinh(_tmp183) - std::sinh(_tmp184)) +
               _tmp180 * (-Scalar(1.0) * _tmp187 * std::cosh(_tmp184) -
                          (Scalar(0.1034955) * _tmp181 * _tmp186 +
                           _tmp182 * (_tmp173 * _tmp179 - _tmp180 * _tmp187)) *
                              std::cosh(_tmp183));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
