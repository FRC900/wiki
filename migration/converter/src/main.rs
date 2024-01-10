use std::{
    env::current_dir,
    fs::{self, OpenOptions},
    io::Write,
    path::PathBuf,
};

const IMAGE_BASE: &str = "../../wiki-resources";

fn main() {
    let input = current_dir().unwrap().parent().unwrap().join("wiki-clone");
    let output = current_dir()
        .unwrap()
        .parent()
        .unwrap()
        .join("wiki-converted");

    if !output.exists() {
        fs::create_dir(&output).unwrap();
    }

    for folder in input.read_dir().unwrap() {
        let input = folder.unwrap();
        let output = output.join(input.file_name().to_str().unwrap());

        if !output.exists() {
            fs::create_dir(&output).unwrap();
        }

        // why, macos...
        if input.file_name().to_str().unwrap() == ".DS_Store" {
            continue;
        }
        for file in input.path().read_dir().unwrap() {
            let file = file.unwrap();
            parse_file(
                file.path(),
                output.join(file.path().with_extension("md").file_name().unwrap()),
            );
        }
    }
}

fn parse_file(path: PathBuf, output: PathBuf) {
    let mut file = OpenOptions::new()
        .write(true)
        .create(true)
        .open(output)
        .unwrap();
    file.set_len(0).unwrap();

    let mut contents = fs::read_to_string(path).unwrap();
    contents = contents.replace("\\\\", "\n");
    contents = contents.replace("======", "# ");
    contents = contents.replace("=====", "## ");
    contents = contents.replace("====", "### ");
    contents = contents.replace("===", "#### ");
    contents = contents.replace("==", "##### ");
    contents = contents.replace("**", "\\\\");
    contents = contents.replace("* ", "- ");
    contents = contents.replace("\\\\", "**");
    contents = contents.replace("://", ":\\");
    contents = contents.replace("//", "*");
    contents = contents.replace(":\\", "://");
    contents = contents.replace("''", "`");
    contents = contents.replace("<code bash>", "```bash");
    contents = contents.replace("<code>", "```");
    contents = contents.replace("</code>", "```");

    while let Some(link_start) = contents.find("[[") {
        let link_end = contents.find("]]").unwrap();
        let link = &contents[link_start + 2..link_end];
        let mut parts = link.split('|');

        let url = parts.next().unwrap();
        let new_link = if let Some(text) = parts.next() {
            format!("[{text}]({url})")
        } else {
            format!("[{url}]({url})")
        };

        contents = contents[0..link_start].to_owned() + &new_link + &contents[link_end + 2..];
    }

    while let Some(image_start) = contents.find("{{") {
        let image_end = contents.find("}}").unwrap();
        let link = &contents[image_start + 2..image_end];
        let mut parts = link.split('|');
        let url = parts
            .next()
            .unwrap()
            .split('?')
            .next()
            .unwrap()
            .replace("::", ":")
            .replace(':', "/");
        let url = IMAGE_BASE.to_owned() + url.trim();

        let new_image = if let Some(label) = parts.next() {
            format!("![{label}]({url})")
        } else {
            format!("![]({url})")
        };

        contents = contents[0..image_start].to_owned() + &new_image + &contents[image_end + 2..];
    }

    file.write_all(contents.as_bytes()).unwrap();
}
