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
 * Symbolic function: IK_residual_func_cost2_wrt_fv1_Nl20
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost2WrtFv1Nl20(
    const Scalar fh1, const Scalar fv1, const sym::Rot3<Scalar>& DeltaRot,
    const Eigen::Matrix<Scalar, 3, 1>& position_vector, const sym::Rot3<Scalar>& Rot_init,
    const Scalar epsilon) {
  // Total ops: 596

  // Unused inputs
  (void)epsilon;

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _DeltaRot = DeltaRot.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _Rot_init = Rot_init.Data();

  // Intermediate terms (189)
  const Scalar _tmp0 = Scalar(1.0) / (fh1);
  const Scalar _tmp1 = std::asinh(_tmp0 * fv1);
  const Scalar _tmp2 = Scalar(1.0) * _tmp0 /
                       std::sqrt(Scalar(1 + std::pow(fv1, Scalar(2)) / std::pow(fh1, Scalar(2))));
  const Scalar _tmp3 = _DeltaRot[0] * _Rot_init[3] - _DeltaRot[1] * _Rot_init[2] +
                       _DeltaRot[2] * _Rot_init[1] + _DeltaRot[3] * _Rot_init[0];
  const Scalar _tmp4 = _DeltaRot[0] * _Rot_init[2] + _DeltaRot[1] * _Rot_init[3] -
                       _DeltaRot[2] * _Rot_init[0] + _DeltaRot[3] * _Rot_init[1];
  const Scalar _tmp5 = 2 * _tmp4;
  const Scalar _tmp6 = _tmp3 * _tmp5;
  const Scalar _tmp7 = -_DeltaRot[0] * _Rot_init[1] + _DeltaRot[1] * _Rot_init[0] +
                       _DeltaRot[2] * _Rot_init[3] + _DeltaRot[3] * _Rot_init[2];
  const Scalar _tmp8 = -2 * _DeltaRot[0] * _Rot_init[0] - 2 * _DeltaRot[1] * _Rot_init[1] -
                       2 * _DeltaRot[2] * _Rot_init[2] + 2 * _DeltaRot[3] * _Rot_init[3];
  const Scalar _tmp9 = _tmp7 * _tmp8;
  const Scalar _tmp10 = Scalar(0.20999999999999999) * _tmp6 - Scalar(0.20999999999999999) * _tmp9;
  const Scalar _tmp11 = 2 * _tmp3 * _tmp7;
  const Scalar _tmp12 = _tmp4 * _tmp8;
  const Scalar _tmp13 = _tmp11 + _tmp12;
  const Scalar _tmp14 = -Scalar(0.010999999999999999) * _tmp13;
  const Scalar _tmp15 = -2 * std::pow(_tmp7, Scalar(2));
  const Scalar _tmp16 = 1 - 2 * std::pow(_tmp4, Scalar(2));
  const Scalar _tmp17 = Scalar(0.20999999999999999) * _tmp15 + Scalar(0.20999999999999999) * _tmp16;
  const Scalar _tmp18 = _tmp14 - _tmp17;
  const Scalar _tmp19 = _tmp10 + _tmp18;
  const Scalar _tmp20 = _tmp19 + position_vector(0, 0);
  const Scalar _tmp21 = Scalar(0.20999999999999999) * _tmp6 + Scalar(0.20999999999999999) * _tmp9;
  const Scalar _tmp22 = -_tmp21;
  const Scalar _tmp23 = -2 * std::pow(_tmp3, Scalar(2));
  const Scalar _tmp24 = Scalar(0.20999999999999999) * _tmp15 +
                        Scalar(0.20999999999999999) * _tmp23 + Scalar(0.20999999999999999);
  const Scalar _tmp25 = _tmp5 * _tmp7;
  const Scalar _tmp26 = _tmp3 * _tmp8;
  const Scalar _tmp27 = _tmp25 - _tmp26;
  const Scalar _tmp28 = Scalar(0.010999999999999999) * _tmp27;
  const Scalar _tmp29 = -_tmp28;
  const Scalar _tmp30 = _tmp24 + _tmp29;
  const Scalar _tmp31 = _tmp22 + _tmp30;
  const Scalar _tmp32 = _tmp31 + position_vector(1, 0);
  const Scalar _tmp33 = Scalar(9.6622558468725703) * fh1;
  const Scalar _tmp34 = -_tmp10;
  const Scalar _tmp35 = _tmp14 + _tmp17;
  const Scalar _tmp36 = _tmp34 + _tmp35;
  const Scalar _tmp37 = Scalar(1.0) * _tmp36;
  const Scalar _tmp38 = -_tmp24;
  const Scalar _tmp39 = _tmp21 + _tmp38;
  const Scalar _tmp40 = _tmp29 + _tmp39;
  const Scalar _tmp41 = Scalar(1.0) * _tmp40;
  const Scalar _tmp42 = -_tmp41;
  const Scalar _tmp43 = _tmp21 + _tmp30;
  const Scalar _tmp44 = _tmp42 + _tmp43;
  const Scalar _tmp45 = _tmp22 + _tmp29 + _tmp38;
  const Scalar _tmp46 = Scalar(1.0) / (_tmp42 + _tmp45);
  const Scalar _tmp47 = _tmp18 + _tmp34;
  const Scalar _tmp48 = _tmp46 * (_tmp37 - _tmp47);
  const Scalar _tmp49 = _tmp44 * _tmp48;
  const Scalar _tmp50 = _tmp10 + _tmp35;
  const Scalar _tmp51 = Scalar(1.0) / (_tmp37 - _tmp49 - _tmp50);
  const Scalar _tmp52 = Scalar(1.0) * _tmp51;
  const Scalar _tmp53 = _tmp45 + position_vector(1, 0);
  const Scalar _tmp54 = _tmp53 + Scalar(8.3196563700000006);
  const Scalar _tmp55 = _tmp47 + position_vector(0, 0);
  const Scalar _tmp56 = _tmp55 + Scalar(1.9874742000000001);
  const Scalar _tmp57 = std::pow(Scalar(std::pow(_tmp54, Scalar(2)) + std::pow(_tmp56, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp58 = _tmp54 * _tmp57;
  const Scalar _tmp59 = _tmp56 * _tmp57;
  const Scalar _tmp60 = _tmp40 + position_vector(1, 0);
  const Scalar _tmp61 = _tmp60 + Scalar(8.3888750099999996);
  const Scalar _tmp62 = _tmp36 + position_vector(0, 0);
  const Scalar _tmp63 = _tmp62 + Scalar(-2.5202214700000001);
  const Scalar _tmp64 =
      std::sqrt(Scalar(std::pow(_tmp61, Scalar(2)) + std::pow(_tmp63, Scalar(2))));
  const Scalar _tmp65 = Scalar(1.0) / (_tmp64);
  const Scalar _tmp66 = Scalar(1.0) / (_tmp63);
  const Scalar _tmp67 = _tmp64 * _tmp66;
  const Scalar _tmp68 = _tmp67 * (_tmp36 * _tmp61 * _tmp65 - _tmp40 * _tmp63 * _tmp65);
  const Scalar _tmp69 = _tmp45 * _tmp59 - _tmp47 * _tmp58 + _tmp59 * _tmp68;
  const Scalar _tmp70 = _tmp61 * _tmp66;
  const Scalar _tmp71 = Scalar(1.0) / (-_tmp58 + _tmp59 * _tmp70);
  const Scalar _tmp72 = Scalar(1.0) * _tmp71;
  const Scalar _tmp73 = Scalar(0.20999999999999999) * _tmp25 + Scalar(0.20999999999999999) * _tmp26;
  const Scalar _tmp74 = -_tmp73;
  const Scalar _tmp75 =
      -Scalar(0.010999999999999999) * _tmp16 - Scalar(0.010999999999999999) * _tmp23;
  const Scalar _tmp76 = Scalar(0.20999999999999999) * _tmp11 - Scalar(0.20999999999999999) * _tmp12;
  const Scalar _tmp77 = _tmp75 + _tmp76;
  const Scalar _tmp78 = _tmp74 + _tmp77;
  const Scalar _tmp79 = _tmp75 - _tmp76;
  const Scalar _tmp80 = _tmp74 + _tmp79;
  const Scalar _tmp81 = _tmp59 * _tmp78 - _tmp59 * _tmp80;
  const Scalar _tmp82 = _tmp70 * _tmp78;
  const Scalar _tmp83 = _tmp58 * _tmp80 - _tmp59 * _tmp82;
  const Scalar _tmp84 = _tmp72 * _tmp83;
  const Scalar _tmp85 = _tmp48 * _tmp84 - _tmp72 * _tmp81;
  const Scalar _tmp86 = _tmp43 + position_vector(1, 0);
  const Scalar _tmp87 = _tmp86 + Scalar(-4.7752063900000001);
  const Scalar _tmp88 = _tmp50 + position_vector(0, 0);
  const Scalar _tmp89 = _tmp88 + Scalar(-2.71799795);
  const Scalar _tmp90 = std::pow(Scalar(std::pow(_tmp87, Scalar(2)) + std::pow(_tmp89, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp91 = _tmp89 * _tmp90;
  const Scalar _tmp92 = _tmp87 * _tmp90;
  const Scalar _tmp93 = _tmp70 * _tmp91 - _tmp92;
  const Scalar _tmp94 = _tmp71 * _tmp93;
  const Scalar _tmp95 = _tmp73 + _tmp77;
  const Scalar _tmp96 = -_tmp82 * _tmp91 - _tmp83 * _tmp94 + _tmp92 * _tmp95;
  const Scalar _tmp97 = -_tmp48 * _tmp96 + _tmp78 * _tmp91 - _tmp81 * _tmp94 - _tmp91 * _tmp95;
  const Scalar _tmp98 = Scalar(1.0) / (_tmp97);
  const Scalar _tmp99 = _tmp43 * _tmp91 - _tmp50 * _tmp92 + _tmp68 * _tmp91 - _tmp69 * _tmp94;
  const Scalar _tmp100 = _tmp98 * _tmp99;
  const Scalar _tmp101 = Scalar(1.0) / (_tmp99);
  const Scalar _tmp102 = _tmp101 * _tmp97;
  const Scalar _tmp103 = _tmp102 * (-_tmp100 * _tmp85 - _tmp69 * _tmp72);
  const Scalar _tmp104 = _tmp44 * _tmp51;
  const Scalar _tmp105 = _tmp98 * (_tmp103 + _tmp85);
  const Scalar _tmp106 = _tmp103 * _tmp104 - _tmp105 * _tmp96 - _tmp84;
  const Scalar _tmp107 = Scalar(1.0) * _tmp46;
  const Scalar _tmp108 = _tmp20 + Scalar(1.79662371);
  const Scalar _tmp109 = _tmp32 + Scalar(-4.8333311099999996);
  const Scalar _tmp110 =
      std::pow(Scalar(std::pow(_tmp108, Scalar(2)) + std::pow(_tmp109, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp111 = _tmp109 * _tmp110;
  const Scalar _tmp112 = _tmp111 * fh1;
  const Scalar _tmp113 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp114 = _tmp37 + _tmp41 * _tmp48;
  const Scalar _tmp115 = _tmp114 * _tmp51;
  const Scalar _tmp116 = 0;
  const Scalar _tmp117 = _tmp46 * (-_tmp115 * _tmp44 - _tmp116 * _tmp96 + _tmp42);
  const Scalar _tmp118 = -Scalar(1.0) * _tmp114 * _tmp52 - Scalar(1.0) * _tmp117 + Scalar(1.0);
  const Scalar _tmp119 = fh1 * (_tmp73 + _tmp79);
  const Scalar _tmp120 = -_tmp111 * _tmp119 - Scalar(5.1796800000000003) * _tmp27 - _tmp31 * fv1;
  const Scalar _tmp121 = _tmp48 * _tmp52;
  const Scalar _tmp122 = _tmp49 * _tmp52 + Scalar(1.0);
  const Scalar _tmp123 = -Scalar(1.0) * _tmp107 * _tmp122 + Scalar(1.0) * _tmp121;
  const Scalar _tmp124 = Scalar(1.0) * _tmp101;
  const Scalar _tmp125 = _tmp102 * _tmp52;
  const Scalar _tmp126 = -_tmp124 * _tmp96 + _tmp125 * _tmp44;
  const Scalar _tmp127 = _tmp108 * _tmp110;
  const Scalar _tmp128 = fh1 * (_tmp111 * _tmp19 - _tmp127 * _tmp31);
  const Scalar _tmp129 = _tmp119 * _tmp127 + Scalar(5.1796800000000003) * _tmp13 + _tmp19 * fv1;
  const Scalar _tmp130 = _tmp44 * _tmp46;
  const Scalar _tmp131 = Scalar(1.0) * _tmp130 * _tmp52 - Scalar(1.0) * _tmp52;
  const Scalar _tmp132 = _tmp70 * _tmp71;
  const Scalar _tmp133 = _tmp132 * _tmp83 + _tmp82;
  const Scalar _tmp134 = _tmp132 * _tmp81 - _tmp133 * _tmp48 - _tmp78;
  const Scalar _tmp135 = _tmp102 * (-_tmp100 * _tmp134 + _tmp132 * _tmp69 - _tmp68);
  const Scalar _tmp136 = _tmp98 * (_tmp134 + _tmp135);
  const Scalar _tmp137 = _tmp104 * _tmp135 + _tmp133 - _tmp136 * _tmp96;
  const Scalar _tmp138 = _tmp127 * fh1;
  const Scalar _tmp139 =
      Scalar(1.0) * _tmp112 * (_tmp103 * _tmp52 - _tmp106 * _tmp107) + _tmp113 * _tmp118 +
      _tmp120 * _tmp123 + Scalar(1.0) * _tmp128 * (-_tmp107 * _tmp126 + _tmp125) +
      _tmp129 * _tmp131 + Scalar(1.0) * _tmp138 * (-_tmp107 * _tmp137 + _tmp135 * _tmp52);
  const Scalar _tmp140 = _tmp101 * _tmp72 * _tmp93;
  const Scalar _tmp141 = -_tmp105 * _tmp93 + Scalar(1.0);
  const Scalar _tmp142 = _tmp59 * _tmp71;
  const Scalar _tmp143 = -_tmp136 * _tmp93 - _tmp70;
  const Scalar _tmp144 = _tmp116 * _tmp94;
  const Scalar _tmp145 = _tmp67 * (_tmp116 * _tmp91 - _tmp144 * _tmp59);
  const Scalar _tmp146 = -_tmp112 * _tmp67 * (_tmp105 * _tmp91 + _tmp141 * _tmp142) -
                         _tmp113 * _tmp145 -
                         _tmp128 * _tmp67 * (_tmp124 * _tmp91 - _tmp140 * _tmp59) -
                         _tmp138 * _tmp67 * (_tmp136 * _tmp91 + _tmp142 * _tmp143 + Scalar(1.0));
  const Scalar _tmp147 = Scalar(1.0) / (_tmp146);
  const Scalar _tmp148 = std::asinh(_tmp139 * _tmp147);
  const Scalar _tmp149 = Scalar(9.6622558468725703) * _tmp146;
  const Scalar _tmp150 =
      -_tmp148 * _tmp149 -
      Scalar(8.3888750099999996) *
          std::sqrt(
              Scalar(Scalar(0.090254729040973036) *
                         std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp62), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp60 - 1), Scalar(2))));
  const Scalar _tmp151 = Scalar(0.1034955) * _tmp147;
  const Scalar _tmp152 = _tmp150 * _tmp151;
  const Scalar _tmp153 = Scalar(1.0) * _tmp148;
  const Scalar _tmp154 = Scalar(9.6622558468725703) * _tmp145;
  const Scalar _tmp155 = std::pow(_tmp146, Scalar(-2));
  const Scalar _tmp156 = _tmp28 + _tmp39;
  const Scalar _tmp157 = _tmp145 * _tmp155;
  const Scalar _tmp158 =
      (-_tmp139 * _tmp157 + _tmp147 * (-_tmp118 + _tmp123 * _tmp156 + _tmp131 * _tmp19)) /
      std::sqrt(Scalar(std::pow(_tmp139, Scalar(2)) * _tmp155 + 1));
  const Scalar _tmp159 = _tmp122 * _tmp46;
  const Scalar _tmp160 = _tmp129 * _tmp52;
  const Scalar _tmp161 = _tmp106 * _tmp112 * _tmp46 + _tmp113 * _tmp117 + _tmp120 * _tmp159 +
                         _tmp126 * _tmp128 * _tmp46 - _tmp130 * _tmp160 +
                         _tmp137 * _tmp138 * _tmp46;
  const Scalar _tmp162 = _tmp113 * _tmp116;
  const Scalar _tmp163 = _tmp112 * _tmp141 * _tmp71 - _tmp128 * _tmp140 +
                         _tmp138 * _tmp143 * _tmp71 - _tmp162 * _tmp94;
  const Scalar _tmp164 = Scalar(1.0) / (_tmp163);
  const Scalar _tmp165 = std::asinh(_tmp161 * _tmp164);
  const Scalar _tmp166 = Scalar(1.0) * _tmp165;
  const Scalar _tmp167 = _tmp19 * _tmp52;
  const Scalar _tmp168 = std::pow(_tmp163, Scalar(-2));
  const Scalar _tmp169 = _tmp144 * _tmp168;
  const Scalar _tmp170 =
      (-_tmp161 * _tmp169 + _tmp164 * (-_tmp117 - _tmp130 * _tmp167 + _tmp156 * _tmp159)) /
      std::sqrt(Scalar(std::pow(_tmp161, Scalar(2)) * _tmp168 + 1));
  const Scalar _tmp171 = Scalar(9.6622558468725703) * _tmp163;
  const Scalar _tmp172 = Scalar(9.6622558468725703) * _tmp116;
  const Scalar _tmp173 = _tmp172 * _tmp94;
  const Scalar _tmp174 = Scalar(0.1034955) * _tmp164;
  const Scalar _tmp175 =
      -_tmp165 * _tmp171 -
      Scalar(8.3196563700000006) *
          std::sqrt(
              Scalar(std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp53 - 1), Scalar(2)) +
                     Scalar(0.057067943376852184) *
                         std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp55 - 1), Scalar(2))));
  const Scalar _tmp176 = _tmp174 * _tmp175;
  const Scalar _tmp177 = _tmp105 * _tmp112 + _tmp124 * _tmp128 + _tmp136 * _tmp138 + _tmp162;
  const Scalar _tmp178 = Scalar(1.0) / (_tmp177);
  const Scalar _tmp179 = -_tmp103 * _tmp112 * _tmp51 + _tmp113 * _tmp115 - _tmp120 * _tmp121 -
                         _tmp125 * _tmp128 - _tmp135 * _tmp138 * _tmp51 + _tmp160;
  const Scalar _tmp180 = std::asinh(_tmp178 * _tmp179);
  const Scalar _tmp181 = Scalar(1.0) * _tmp180;
  const Scalar _tmp182 = Scalar(9.6622558468725703) * _tmp177;
  const Scalar _tmp183 =
      -_tmp180 * _tmp182 -
      Scalar(4.7752063900000001) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp86), Scalar(2)) +
                     Scalar(0.32397683292140877) *
                         std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp88), Scalar(2))));
  const Scalar _tmp184 = Scalar(0.1034955) * _tmp178;
  const Scalar _tmp185 = _tmp183 * _tmp184;
  const Scalar _tmp186 = std::pow(_tmp177, Scalar(-2));
  const Scalar _tmp187 = _tmp116 * _tmp186;
  const Scalar _tmp188 = (_tmp178 * (-_tmp115 - _tmp121 * _tmp156 + _tmp167) + _tmp179 * _tmp187) /
                         std::sqrt(Scalar(std::pow(_tmp179, Scalar(2)) * _tmp186 + 1));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      _tmp33 *
      (-_tmp2 * std::cosh(Scalar(1.0) * _tmp1) +
       _tmp2 * std::cosh(
                   Scalar(0.1034955) * _tmp0 *
                   (-_tmp1 * _tmp33 -
                    Scalar(4.8333311099999996) *
                        std::sqrt(Scalar(
                            std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp32), Scalar(2)) +
                            Scalar(0.13817235445745474) *
                                std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp20 - 1),
                                         Scalar(2)))))));
  _res(1, 0) = _tmp149 * (-Scalar(1.0) * _tmp158 * std::cosh(_tmp153) -
                          (-Scalar(0.1034955) * _tmp150 * _tmp157 +
                           _tmp151 * (-_tmp148 * _tmp154 - _tmp149 * _tmp158)) *
                              std::cosh(_tmp152)) +
               _tmp154 * (-std::sinh(_tmp152) - std::sinh(_tmp153));
  _res(2, 0) = _tmp171 * (-Scalar(1.0) * _tmp170 * std::cosh(_tmp166) -
                          (-Scalar(0.1034955) * _tmp169 * _tmp175 +
                           _tmp174 * (-_tmp165 * _tmp173 - _tmp170 * _tmp171)) *
                              std::cosh(_tmp176)) +
               _tmp173 * (-std::sinh(_tmp166) - std::sinh(_tmp176));
  _res(3, 0) = -_tmp172 * (-std::sinh(_tmp181) - std::sinh(_tmp185)) +
               _tmp182 * (-Scalar(1.0) * _tmp188 * std::cosh(_tmp181) -
                          (Scalar(0.1034955) * _tmp183 * _tmp187 +
                           _tmp184 * (_tmp172 * _tmp180 - _tmp182 * _tmp188)) *
                              std::cosh(_tmp185));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
