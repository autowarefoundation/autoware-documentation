// The video in the hero of the homepage plays on its own, without sound, unless the reader asks for less motion.
// It has no autoplay attribute, because a browser plays such a video even when the reader asks for less motion.

document$.subscribe(() => {
  const video = document.querySelector(".aw-hero__video video");
  if (!video || matchMedia("(prefers-reduced-motion: reduce)").matches) return;
  video.play().catch(() => {});
});
