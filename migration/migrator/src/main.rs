use {
    reqwest::{cookie::Jar, Client, ClientBuilder},
    scraper::{Html, Selector},
    std::{
        collections::HashMap,
        env,
        fs::{self, OpenOptions},
        io::Write,
        sync::Arc,
        time::Duration,
    },
    tokio::task::JoinSet,
};

const COOKIES: &str = include_str!("../cookies.txt");
const URL: &str = "https://wiki.team900.org";

#[tokio::main]
async fn main() {
    let url = URL.parse().unwrap();
    let cookie_jar = Jar::default();

    for cookie in COOKIES.lines() {
        cookie_jar.add_cookie_str(cookie, &url);
    }

    let client = ClientBuilder::default()
        .cookie_provider(Arc::new(cookie_jar))
        .timeout(Duration::from_secs(5))
        .build()
        .unwrap();

    let sitemap_src = client
        .get(url.join("doku.php?do=index").unwrap())
        .send()
        .await
        .unwrap()
        .text()
        .await
        .unwrap();

    let page_map = build_pages_list(&sitemap_src, &client).await;

    let cwd = env::current_dir().unwrap();
    let output = cwd.join("output");
    if !output.exists() {
        fs::create_dir(&output).unwrap();
    }
    for (page_name, mut page_data) in page_map {
        let folder = output.join(page_name);
        if !folder.exists() {
            fs::create_dir(&folder).unwrap();
        }

        if let Some(url) = page_data.url {
            page_data.children.push(("index".to_string(), url));
        }

        let mut tasks = JoinSet::new();
        for (name, url) in page_data.children {
            let path = folder.join(name).with_extension("html");
            let client = client.clone();
            tasks.spawn(async move {
                println!("{url}");
                let mut file = OpenOptions::new()
                    .write(true)
                    .create(true)
                    .open(path)
                    .unwrap();
                let page = client
                    .get(URL.to_string() + &url)
                    .send()
                    .await
                    .unwrap()
                    .bytes()
                    .await
                    .unwrap();
                file.write_all(&page).unwrap();
            });
        }
    }
}

async fn build_pages_list(sitemap_src: &str, client: &Client) -> HashMap<String, SiteMapPage> {
    let sitemap_page = Html::parse_document(sitemap_src);

    let mut page_map: HashMap<String, SiteMapPage> = HashMap::new();

    // The sitemap has 2 types of links: Direct page links, and links that open up a list of subpages.
    // Direct page links have the `wikilink1` class, while subpage links have the `idx_dir` class.
    // Not all sections have subpages, and not all sections have main pages, so both are optional.

    // Get direct page links
    sitemap_page
        .select(&Selector::parse(".wikilink1").unwrap())
        .map(|link| link.value())
        .for_each(|link| {
            let page_name = link.attr("data-wiki-id").unwrap().to_string();
            let page_link = link.attr("href").unwrap().to_string();
            page_map.entry(page_name).or_default().url = Some(page_link);
        });

    // Get subpage links
    let mut subpage_tasks = sitemap_page
        .select(&Selector::parse(".idx_dir").unwrap())
        .map(|link| link.value())
        .map(|link| {
            let page_name = link.attr("title").unwrap().to_string();
            let page_map_url = link.attr("href").unwrap().to_string();
            (page_name, page_map_url)
        })
        .fold(JoinSet::new(), |mut set, (page_name, page_map_url)| {
            let client = client.clone();
            set.spawn(parse_subpages(client, page_name, page_map_url));
            set
        });
    while !subpage_tasks.is_empty() {
        let (page_name, subpage_urls) = subpage_tasks.join_next().await.unwrap().unwrap();
        page_map.entry(page_name).or_default().children = subpage_urls;
    }

    page_map
}

async fn parse_subpages(
    client: Client,
    page_name: String,
    page_map_url: String,
) -> (String, Vec<(String, String)>) {
    let page_source = Html::parse_document(
        &client
            .get(URL.to_string() + &page_map_url)
            .send()
            .await
            .unwrap()
            .text()
            .await
            .unwrap(),
    );

    let mut subpage_urls = Vec::new();

    page_source
        .select(&Selector::parse(".level2").unwrap())
        .map(|link| {
            // Get the grandchild element, which is the actual <a> tag/link
            link.children()
                .next()
                .unwrap()
                .children()
                .next()
                .unwrap()
                .value()
                .as_element()
                .unwrap()
        })
        .for_each(|subpage_link| {
            let subpage_name = subpage_link.attr("title").unwrap();
            let subpage_url = subpage_link.attr("href").unwrap();

            subpage_urls.push((subpage_name.to_string(), subpage_url.to_string()))
        });

    (page_name, subpage_urls)
}

#[derive(Default)]
struct SiteMapPage {
    /// A link to the page
    url: Option<String>,
    /// Links to the page's subpages - <(subpage name, subpage url)>
    children: Vec<(String, String)>,
}
