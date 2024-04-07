set(D ./codebook)

set(codec2_srcs "codec2.c" "interp.c" "lpc.c" "lsp.c" "nlp.c" "phase.c" "postfilter.c" "quantise.c" "sine.c"
  "kiss_fft.c" "kiss_fftr.c" "newamp1.c" "mbest.c" "codec2_fft.c" "pack.c"
  "${D}/codebook.c" "${D}/codebookd.c" "${D}/codebookge.c" "${D}/codebookjmv.c"
  "${D}/codebooknewamp1.c" "${D}/codebooknewamp1_energy.c" "${D}/codebooknewamp2.c" "${D}/codebooknewamp2_energy.c"
)

set(codec2_reqs "")


idf_component_register(
  SRCS ${codec2_srcs}
  INCLUDE_DIRS "include"
  REQUIRES ${codec2_reqs}
  PRIV_REQUIRES "arch"
)
