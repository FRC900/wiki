use std::{
    env::current_dir,
    fs::{self, OpenOptions},
    io::Write,
    path::PathBuf,
};

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
    let contents = fs::read_to_string(path).unwrap();
    let contents = contents.replace("=====", "#");
    let contents = contents.replace("- ", "1. ");
    let contents = contents.replace("* ", "- ");
    let contents = contents.replace("//", "*");
    let contents = contents.replace("''", "`");
    let contents = contents.replace("<code bash>", "```bash");
    let contents = contents.replace("<code>", "```");
    let contents = contents.replace("</code>", "```");

    file.write_all(contents.as_bytes()).unwrap();
}
