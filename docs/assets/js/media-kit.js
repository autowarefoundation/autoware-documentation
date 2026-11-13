// The media kit page (home/media-kit/). The page links each logo as an SVG file. This script adds a PNG download
// next to each SVG link, a button that downloads all logos as one ZIP file, and a copy button on each color code.
// The PNG files and the ZIP file are made in the browser, so the repository holds only the SVG files.

const AW_PNG_SIZE = 2048; // the longer side of each PNG file, in pixels

// Make a PNG file from an SVG file. The SVG gets a width and a height first, because Firefox draws an SVG image
// on a canvas only if the image has a size of its own.
async function awSvgToPng(url) {
  const text = await (await fetch(url)).text();
  const [, , w, h] = text
    .match(/viewBox="([^"]+)"/)[1]
    .split(/[\s,]+/)
    .map(Number);
  const scale = AW_PNG_SIZE / Math.max(w, h);
  const width = Math.round(w * scale);
  const height = Math.round(h * scale);
  const sized = text.replace(/<svg\b/, `<svg width="${width}" height="${height}"`);
  const image = new Image();
  image.src = URL.createObjectURL(new Blob([sized], { type: "image/svg+xml" }));
  await image.decode();
  const canvas = document.createElement("canvas");
  canvas.width = width;
  canvas.height = height;
  canvas.getContext("2d").drawImage(image, 0, 0, width, height);
  URL.revokeObjectURL(image.src);
  return new Promise((resolve) => canvas.toBlob(resolve, "image/png"));
}

function awSave(blob, name) {
  const link = document.createElement("a");
  link.href = URL.createObjectURL(blob);
  link.download = name;
  link.click();
  setTimeout(() => URL.revokeObjectURL(link.href), 1000);
}

const AW_CRC_TABLE = Array.from({ length: 256 }, (_, n) => {
  let c = n;
  for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1;
  return c >>> 0;
});

function awCrc32(bytes) {
  let crc = 0xffffffff;
  for (const b of bytes) crc = AW_CRC_TABLE[(crc ^ b) & 0xff] ^ (crc >>> 8);
  return (crc ^ 0xffffffff) >>> 0;
}

// Make a ZIP file that stores the files without compression. SVG and PNG files are small or already compressed.
function awZip(files) {
  const encoder = new TextEncoder();
  const parts = [];
  const central = [];
  let offset = 0;
  for (const file of files) {
    const name = encoder.encode(file.name);
    const crc = awCrc32(file.data);
    const header = (signature, extra) => {
      const view = new DataView(new ArrayBuffer(extra ? 46 : 30));
      let i = 0;
      const u16 = (v) => (view.setUint16(i, v, true), (i += 2));
      const u32 = (v) => (view.setUint32(i, v, true), (i += 4));
      u32(signature);
      if (extra) u16(20); // version made by
      u16(20); // version needed
      u16(0x0800); // UTF-8 names
      u16(0); // stored
      u16(0); // time
      u16(0x21); // date: 1980-01-01
      u32(crc);
      u32(file.data.length);
      u32(file.data.length);
      u16(name.length);
      u16(0); // extra field
      if (extra) {
        u16(0); // comment
        u16(0); // disk
        u16(0); // internal attributes
        u32(0); // external attributes
        u32(offset);
      }
      return new Uint8Array(view.buffer);
    };
    const local = header(0x04034b50, false);
    parts.push(local, name, file.data);
    central.push(header(0x02014b50, true), name);
    offset += local.length + name.length + file.data.length;
  }
  const size = central.reduce((sum, part) => sum + part.length, 0);
  const end = new DataView(new ArrayBuffer(22));
  end.setUint32(0, 0x06054b50, true);
  end.setUint16(8, files.length, true);
  end.setUint16(10, files.length, true);
  end.setUint32(12, size, true);
  end.setUint32(16, offset, true);
  return new Blob([...parts, ...central, end], { type: "application/zip" });
}

document$.subscribe(() => {
  const kit = document.querySelector(".aw-kit");
  if (!kit) return;

  const svgLinks = [...kit.querySelectorAll('a[download][href$=".svg"]')];
  for (const svgLink of svgLinks) {
    const png = document.createElement("a");
    png.href = svgLink.href;
    png.textContent = "PNG";
    png.className = "aw-kit__png";
    png.addEventListener("click", async (event) => {
      event.preventDefault();
      const name = svgLink.getAttribute("href").split("/").pop().replace(".svg", ".png");
      awSave(await awSvgToPng(svgLink.href), name);
    });
    svgLink.after(png);
  }

  const all = document.querySelector(".aw-kit-all");
  if (all) {
    all.hidden = false;
    all.addEventListener("click", async () => {
      const label = all.textContent;
      all.textContent = "Preparing the ZIP file...";
      all.disabled = true;
      const files = [];
      for (const link of svgLinks) {
        const name = link.getAttribute("href").split("/").pop();
        const svg = new Uint8Array(await (await fetch(link.href)).arrayBuffer());
        const png = new Uint8Array(await (await awSvgToPng(link.href)).arrayBuffer());
        files.push({ name: `autoware-logos/svg/${name}`, data: svg });
        files.push({ name: `autoware-logos/png/${name.replace(".svg", ".png")}`, data: png });
      }
      awSave(awZip(files), "autoware-logos.zip");
      all.textContent = label;
      all.disabled = false;
    });
  }

  for (const code of document.querySelectorAll(".aw-swatch__info code")) {
    const button = document.createElement("button");
    button.type = "button";
    button.className = "aw-copy";
    button.title = "Copy";
    button.addEventListener("click", async () => {
      await navigator.clipboard.writeText(code.textContent);
      button.classList.add("aw-copy--done");
      setTimeout(() => button.classList.remove("aw-copy--done"), 1200);
    });
    code.replaceWith(button);
    button.append(code);
  }
});
